#include <string>
#include <cstdio>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

#include <hailo/hailort.h>
#include <hailo/hailort_common.hpp>
#include <hailo/vdevice.hpp>
#include <hailo/infer_model.hpp>

static const std::string HEF_PATH = "models/hailo/fastdepth--224x224.hef";
//static const std::string HEF_PATH = "depth_anything/depth_anything_v2--224x224.hef";
static const float CONF_THRESHOLD    = 0.4f;
static const float NMS_IOU_THRESHOLD = 0.45f;

int main(int argc, char* argv[]) {
    std::string image_path = (argc > 1) ? argv[1] : "test_image.jpeg";

    const cv::Mat orig = cv::imread(image_path);
    if (orig.empty()) {
        fprintf(stderr, "Failed to load image: %s\n", image_path.c_str());
        return 1;
    }

    using namespace hailort;

    // ── 1. Create VDevice ───────────────────────────────────────────────────
    auto vdevice_exp = VDevice::create();
    if (!vdevice_exp) {
        fprintf(stderr, "Failed to create VDevice, status = %d\n",
                (int)vdevice_exp.status());
        return 1;
    }
    auto vdevice = vdevice_exp.release();

    // ── 2. Load model ───────────────────────────────────────────────────────
    auto infer_model_exp = vdevice->create_infer_model(HEF_PATH);
    if (!infer_model_exp) {
        fprintf(stderr, "Failed to create InferModel, status = %d\n",
                (int)infer_model_exp.status());
        return 1;
    }
    auto infer_model = infer_model_exp.release();

    // ── 3. Configure output format ──────────────────────────────────────────
    // Depth models output a dense float map — no NMS, just raw floats
    infer_model->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);

    auto configured_exp = infer_model->configure();
    if (!configured_exp) {
        fprintf(stderr, "Failed to configure model, status = %d\n",
                (int)configured_exp.status());
        return 1;
    }
    auto configured = std::make_shared<ConfiguredInferModel>(configured_exp.release());

    // ── 4. Read input/output shapes from the model ──────────────────────────
    const std::string input_name  = infer_model->get_input_names()[0];
    const std::string output_name = infer_model->get_output_names()[0];

    auto input_shape  = infer_model->input(input_name)->shape();
    auto output_shape = infer_model->output(output_name)->shape();

    // Shape is [height, width, channels] in HailoRT
    const int input_h  = input_shape.height;
    const int input_w  = input_shape.width;
    const int output_h = output_shape.height;
    const int output_w = output_shape.width;

    printf("Input  name: %s  shape: %d x %d\n", input_name.c_str(), input_h, input_w);
    printf("Output name: %s  shape: %d x %d\n", output_name.c_str(), output_h, output_w);

    cv::Mat resized;
    cv::resize(orig, resized, cv::Size(input_w, input_h));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    printf("Preprocessed to %d x %d\n", input_w, input_h);

    // ── 6. Create bindings ──────────────────────────────────────────────────
    auto bindings_exp = configured->create_bindings();
    if (!bindings_exp) {
        fprintf(stderr, "Failed to create bindings, status = %d\n",
                (int)bindings_exp.status());
        return 1;
    }
    auto bindings = std::move(bindings_exp.release());

    // ── 7. Bind input buffer ────────────────────────────────────────────────
    size_t input_size = infer_model->input(input_name)->get_frame_size();
    auto input_status = bindings.input(input_name)->set_buffer(
        MemoryView(rgb.data, input_size));
    if (input_status != HAILO_SUCCESS) {
        fprintf(stderr, "Failed to bind input, status = %d\n", (int)input_status);
        return 1;
    }

    // ── 8. Bind output buffer ───────────────────────────────────────────────
    size_t output_size = infer_model->output(output_name)->get_frame_size();
    std::vector<float> output_buf(output_size / sizeof(float), 0.f);
    auto output_status = bindings.output(output_name)->set_buffer(
        MemoryView(output_buf.data(), output_size));
    if (output_status != HAILO_SUCCESS) {
        fprintf(stderr, "Failed to bind output, status = %d\n", (int)output_status);
        return 1;
    }

    printf("Input  buffer: %zu bytes\n", input_size);
    printf("Output buffer: %zu bytes (%zu floats)\n", output_size, output_buf.size());

    // ── 9. Run inference ────────────────────────────────────────────────────
    auto status = configured->run(bindings, std::chrono::milliseconds(1000));
    if (status != HAILO_SUCCESS) {
        fprintf(stderr, "Inference failed, status = %d\n", (int)status);
        return 1;
    }
    printf("Inference OK\n");

    // ── 10. Wrap output as a float Mat ──────────────────────────────────────
    // output_buf is a flat [output_h * output_w] array of float32 depth values
    cv::Mat depth_map(output_h, output_w, CV_32FC1, output_buf.data());

    // ── 11. Normalize to 0–255 for visualization ────────────────────────────
    cv::Mat depth_norm;
    cv::normalize(depth_map, depth_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // Apply a colormap so it's easier to read visually
    cv::Mat depth_color;
    cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_MAGMA);

    // Resize colormap back to original image size for display
    cv::Mat depth_display;
    cv::resize(depth_color, depth_display, cv::Size(orig.cols, orig.rows));

    // ── 12. Save output ─────────────────────────────────────────────────────
    cv::imwrite("depth_output.png", depth_display);
    printf("Saved depth_output.png\n");

    // Optional: side-by-side comparison
    cv::Mat comparison;
    cv::hconcat(orig, depth_display, comparison);
    cv::imwrite("comparison.png", comparison);
    printf("Saved comparison.png\n");

    return 0;
}