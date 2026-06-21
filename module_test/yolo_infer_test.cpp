#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <hailo/hailort.h>
#include <hailo/hailort_common.hpp>
#include <hailo/vdevice.hpp>
#include <hailo/infer_model.hpp>

// ── config ────────────────────────────────────────────────────────────────────
static const std::string HEF_PATH   = "models/hailo/yolov8s_h8l.hef";
static const float CONF_THRESHOLD   = 0.4f;
static const float NMS_IOU_THRESHOLD = 0.45f;
static const int   INPUT_SIZE       = 640;

// COCO class names
static const std::vector<std::string> COCO_CLASSES = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
};

// ── helpers ───────────────────────────────────────────────────────────────────
struct Detection {
    std::string label;
    float       score;
    int x1, y1, x2, y2;
};

// Letterbox: resize keeping aspect ratio, pad with grey to INPUT_SIZE x INPUT_SIZE
cv::Mat letterbox(const cv::Mat& src, float& scale, int& pad_x, int& pad_y) {
    int orig_w = src.cols, orig_h = src.rows;
    scale = std::min((float)INPUT_SIZE / orig_w, (float)INPUT_SIZE / orig_h);
    int new_w = (int)(orig_w * scale);
    int new_h = (int)(orig_h * scale);

    cv::Mat resized;
    cv::resize(src, resized, {new_w, new_h});

    cv::Mat padded(INPUT_SIZE, INPUT_SIZE, CV_8UC3, cv::Scalar(114, 114, 114));
    pad_x = (INPUT_SIZE - new_w) / 2;
    pad_y = (INPUT_SIZE - new_h) / 2;
    resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return padded;
}

// ── NMS output decoding ───────────────────────────────────────────────────────
// Hailo NMS float32 layout per class:
//   float32 count                          (number of valid detections)
//   float32[max_bboxes][5]                 (y_min, x_min, y_max, x_max, score)
std::vector<Detection> decode_nms(
    const float*          buf,
    const hailo_nms_shape_t& nms_shape,
    float scale, int pad_x, int pad_y,
    int orig_w, int orig_h)
{
    std::vector<Detection> detections;
    size_t num_classes      = nms_shape.number_of_classes;
    size_t max_bboxes       = nms_shape.max_bboxes_per_class;
    size_t floats_per_class = 1 + max_bboxes * 5; // count + bbox data

    for (size_t cls = 0; cls < num_classes; ++cls) {
        const float* cls_ptr = buf + cls * floats_per_class;
        int count = (int)cls_ptr[0];

        for (int d = 0; d < count; ++d) {
            const float* det = cls_ptr + 1 + d * 5;
            float y_min = det[0], x_min = det[1];
            float y_max = det[2], x_max = det[3];
            float score = det[4];

            if (score < CONF_THRESHOLD) continue;

            // Convert normalised coords → original image pixel coords
            int x1 = (int)((x_min * INPUT_SIZE - pad_x) / scale);
            int y1 = (int)((y_min * INPUT_SIZE - pad_y) / scale);
            int x2 = (int)((x_max * INPUT_SIZE - pad_x) / scale);
            int y2 = (int)((y_max * INPUT_SIZE - pad_y) / scale);
            x1 = std::max(0, x1);  y1 = std::max(0, y1);
            x2 = std::min(orig_w, x2); y2 = std::min(orig_h, y2);

            std::string label = (cls < COCO_CLASSES.size())
                ? COCO_CLASSES[cls]
                : "class_" + std::to_string(cls);

            detections.push_back({label, score, x1, y1, x2, y2});
        }
    }
    return detections;
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::string image_path = (argc > 1) ? argv[1] : "test_image.jpeg";

    // ── 1. Load & letterbox image ─────────────────────────────────────────────
    cv::Mat orig = cv::imread(image_path);
    if (orig.empty()) {
        fprintf(stderr, "Failed to load image: %s\n", image_path.c_str());
        return 1;
    }
    int orig_w = orig.cols, orig_h = orig.rows;

    float scale; int pad_x, pad_y;
    cv::Mat padded = letterbox(orig, scale, pad_x, pad_y);

    // Hailo expects RGB — OpenCV gives BGR by default
    cv::Mat rgb;
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

    // ── 2. Setup Hailo device & model ─────────────────────────────────────────
    using namespace hailort;

    auto vdevice_exp = VDevice::create();
    if (!vdevice_exp) { fprintf(stderr, "Failed to create VDevice\n"); return 1; }
    auto vdevice = vdevice_exp.release();

    auto infer_model_exp = vdevice->create_infer_model(HEF_PATH);
    if (!infer_model_exp) { fprintf(stderr, "Failed to create InferModel\n"); return 1; }
    auto infer_model = infer_model_exp.release();

    // Set NMS thresholds directly on the model before configuring
    infer_model->output()->set_nms_score_threshold(CONF_THRESHOLD);
    infer_model->output()->set_nms_iou_threshold(NMS_IOU_THRESHOLD);
    infer_model->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);

    auto configured_exp = infer_model->configure();
    if (!configured_exp) { fprintf(stderr, "Failed to configure model\n"); return 1; }
    auto configured = std::make_shared<ConfiguredInferModel>(configured_exp.release());

    // ── 3. Allocate I/O buffers & bind ────────────────────────────────────────
    auto bindings_exp = configured->create_bindings();
    if (!bindings_exp) { fprintf(stderr, "Failed to create bindings\n"); return 1; }
    auto bindings = std::move(bindings_exp.release());

    // Input: bind the rgb image data directly (640*640*3 bytes)
    const std::string input_name = infer_model->get_input_names()[0];
    size_t input_size = infer_model->input(input_name)->get_frame_size();
    if (rgb.total() * rgb.elemSize() != input_size) {
        fprintf(stderr, "Image size mismatch: got %zu, need %zu\n",
                rgb.total() * rgb.elemSize(), input_size);
        return 1;
    }
    bindings.input(input_name)->set_buffer(MemoryView(rgb.data, input_size));

    // Output: allocate a flat float buffer
    const std::string output_name = infer_model->get_output_names()[0];
    size_t output_size = infer_model->output(output_name)->get_frame_size();
    std::vector<float> output_buf(output_size / sizeof(float), 0.f);
    bindings.output(output_name)->set_buffer(
        MemoryView(output_buf.data(), output_size));

    // ── 4. Run inference ──────────────────────────────────────────────────────
    auto status = configured->run({std::ref(bindings)}, std::chrono::milliseconds(5000));
    if (status != HAILO_SUCCESS) {
        fprintf(stderr, "Inference failed: %d\n", (int)status);
        return 1;
    }

    // ── 5. Decode NMS output ──────────────────────────────────────────────────
    hailo_nms_shape_t nms_shape = infer_model->output(output_name)->get_nms_shape().release();
    printf("NMS shape: %zu classes, %zu max_bboxes\n",
           (size_t)nms_shape.number_of_classes,
           (size_t)nms_shape.max_bboxes_per_class);

    auto detections = decode_nms(
        output_buf.data(), nms_shape,
        scale, pad_x, pad_y,
        orig_w, orig_h);

    // ── 6. Print & draw results ───────────────────────────────────────────────
    printf("\nFound %zu detection(s) above %.2f confidence:\n\n",
           detections.size(), CONF_THRESHOLD);

    for (auto& det : detections) {
        printf("  %-20s  conf=%.2f  box=[%d,%d,%d,%d]\n",
               det.label.c_str(), det.score,
               det.x1, det.y1, det.x2, det.y2);

        cv::rectangle(orig, {det.x1, det.y1}, {det.x2, det.y2},
                      {0, 255, 0}, 2);
        std::string text = det.label + " " +
                           std::to_string((int)(det.score * 100)) + "%";
        cv::putText(orig, text, {det.x1, det.y1 - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    }
    namespace fs = std::filesystem;
    fs::path dir = "results";

    // create folder if it doesn't exist
    if (!fs::exists(dir))
    {
        fs::create_directories(dir);
    }

    cv::imwrite((dir / "object_detection.png").string(), orig);
    return 0;
}
