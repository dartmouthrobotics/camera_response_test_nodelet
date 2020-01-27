#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <pigpio.h>
#include <memory>
#include <chrono>
#include <thread>
#include <fstream>
#include <pluginlib/class_list_macros.h>

namespace camera_response_test {

const int LED_PIN(23);
const float LED_ON_THRESHOLD(70.0);
const int AFTER_DETECT_PAUSE_FRAMES(15);
const std::string OUTPUT_FILE("/home/ubuntu/ros-catkin-workspace/src/camera_response_test/data/response_test.csv");

const int NUMBER_SAMPLES(1000);

struct DataPoint {
    int led_turned_on_frame;
    int detected_frame;
    float detection_delay_seconds;
    float time_since_last_frame_seconds;
};

class CameraResponseTestNodelet : public nodelet::Nodelet {
public:
    std::unique_ptr<image_transport::ImageTransport> image_transport_ptr;
    image_transport::Subscriber image_subscriber;
    std::string camera_image_topic;
    ros::Time led_gpio_activation_time;
    ros::Time last_frame_time;

    int number_frames_off;
    int current_frame_number;
    int led_turned_on_frame;
    bool data_file_written;
    bool on_detected;

    std::vector<DataPoint> data_points;

    bool led_is_on(const sensor_msgs::ImageConstPtr& image_message) {
        int byte_total = 0;

        size_t start_idx = 0;
        size_t stop_idx = image_message->data.size() / 3;

        for (size_t i = start_idx; i < stop_idx; ++i) {
            byte_total += static_cast<int>(image_message->data[i]);
        }

        float average_activation = static_cast<float>(byte_total) / static_cast<float>(stop_idx - start_idx);

        if (average_activation > LED_ON_THRESHOLD) {
            return true;
        }

        return false;
    }

    void image_callback(const sensor_msgs::ImageConstPtr& image_message) {
        if (data_points.size() >= NUMBER_SAMPLES) {
            if (!data_file_written) {
                std::cout << "All samples collected! Outputting to file and terminating" << std::endl;
                std::cout << "Writing data to output csv " << OUTPUT_FILE << std::endl;

                std::ofstream output_stream(OUTPUT_FILE);
                output_stream << "led_turned_on_frame,detected_frame,detection_delay_seconds,time_since_last_frame_seconds" << std::endl;

                for (auto& data_point : data_points) {
                    output_stream << data_point.led_turned_on_frame << "," << data_point.detected_frame << "," << data_point.detection_delay_seconds << "," << data_point.time_since_last_frame_seconds << std::endl;
                }

                output_stream.close();

                std::cout << "Finished writing output" << std::endl;
                data_file_written = true;
            }

            return;
        }

        ros::Time frame_acquisition_time = ros::Time::now();
        ++current_frame_number;

        if (led_is_on(image_message)) {
            gpioWrite(LED_PIN, 0);
            if (!on_detected) {
                float detection_delay = (frame_acquisition_time - led_gpio_activation_time).toSec();
                std::cout << "Time to sense: " << detection_delay << " number samples " << data_points.size() + 1 << std::endl;

                DataPoint data_point;
                data_point.led_turned_on_frame = led_turned_on_frame;
                data_point.detected_frame = current_frame_number;
                data_point.detection_delay_seconds = detection_delay;
                data_point.time_since_last_frame_seconds = (frame_acquisition_time - last_frame_time).toSec();

                data_points.push_back(data_point);

                on_detected = true;
            }
        } else {
            ++number_frames_off;

            if (number_frames_off % AFTER_DETECT_PAUSE_FRAMES == 0) {
                on_detected = false;
                gpioWrite(LED_PIN, 1);
                led_turned_on_frame = current_frame_number;
                led_gpio_activation_time = ros::Time::now();
            }
        }

        last_frame_time = frame_acquisition_time;
    }

    ~CameraResponseTestNodelet() {
        gpioTerminate();
    }

    void onInit() {
        number_frames_off = 0;
        current_frame_number = 0;
        led_turned_on_frame = 0;
        data_file_written = false;
        last_frame_time = ros::Time::now();
        on_detected = false;

        if (gpioInitialise() < 0) {
            fprintf(stderr, "pigpio initialisation failed\n");
            return;
        }

        gpioSetMode(LED_PIN, PI_OUTPUT);
        gpioWrite(LED_PIN, 0);

        camera_image_topic = "/camera_array/cam0/image_raw";
        ros::NodeHandle& private_node_handle = getPrivateNodeHandle();
        image_transport_ptr = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(private_node_handle));
        image_subscriber = image_transport_ptr->subscribe(camera_image_topic, 1, &CameraResponseTestNodelet::image_callback, this); 
    }
}; // class CameraResponseTestNodelet

PLUGINLIB_EXPORT_CLASS(camera_response_test::CameraResponseTestNodelet, nodelet::Nodelet);
} // namespace camera_response_test

