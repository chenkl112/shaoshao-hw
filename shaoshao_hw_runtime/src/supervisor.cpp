#include "shaoshao_runtime/manager.hpp"
#include "shaoshao_runtime/rpc.hpp"
#include "shaoshao_runtime/tasks.hpp"
#include "shaoshao_runtime/ros.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <csignal>
#include <iostream>

namespace hw_runtime {

class Supervisor {
public:
  Supervisor(fs::path root, fs::path initial)
      : root_(fs::absolute(std::move(root))), manager_(root_, std::move(initial)),
        tasks_(manager_.maps), node_(std::make_shared<rclcpp::Node>("hw_supervisor")),
        rpc_(root_ / "web-supervisor.sock", [this](const Json::Value& request) {
          refresh();
          return tasks_.dispatch(request, steady());
        }) {
    executor_.add_node(node_);
    save_map_ = node_->create_client<std_srvs::srv::Trigger>("/map_save");
    finalize_index_ = node_->create_client<std_srvs::srv::Trigger>("/hw/finalize_index");
    retry_localization_ = node_->create_client<std_srvs::srv::Trigger>("/hw/localization/retry");
    manager_.trigger = [this](const std::string& service) { trigger(service); };
    tasks_.hooks.mode = [this](const std::string& mode) { request_mode(mode); };
    tasks_.hooks.retry = [this] { trigger("/hw/localization/retry"); };
    localization_ = node_->create_subscription<std_msgs::msg::String>(
        "/hw/localization/status", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          try { tasks_.localization_state(json_parse(msg->data)); } catch (...) {}
        });
    launch_web();
    spin_ = std::jthread([this] { executor_.spin(); });
    refresh();
  }

  ~Supervisor() {
    if (transition_.joinable()) transition_.request_stop();
    executor_.cancel();
    if (spin_.joinable()) spin_.join();
    executor_.remove_node(node_);
  }

  void run() {
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    while (!stopped_) {
      refresh();
      tasks_.tick(steady());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

private:
  static inline std::atomic_bool stopped_{false};
  static void stop_signal(int) { stopped_ = true; }
  fs::path root_;
  Manager manager_;
  Tasks tasks_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::jthread spin_;
  std::jthread transition_;
  std::atomic_bool transition_running_{false};
  Process web_;
  std::mutex transition_mutex_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_map_, finalize_index_, retry_localization_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_;
  RPCServer rpc_;

  void launch_web() {
    auto assets = root_ / "shaoshao/web_panel";
    if (!fs::is_regular_file(assets / "index.html") || !fs::is_regular_file(assets / "stream.js"))
      throw std::runtime_error("Web panel assets are missing");
    auto token_path = root_ / "web.token";
    std::string token;
    if (fs::is_regular_file(token_path)) {
      token = read_text(token_path, 1024);
      while (!token.empty() && std::isspace(static_cast<unsigned char>(token.back()))) token.pop_back();
    } else {
      token = random_token();
      atomic_write(token_path, token + "\n");
      chmod(token_path.c_str(), 0600);
    }
    if (token.empty()) throw std::runtime_error("Web access token is empty");
    setenv("SCURM_WEB_TOKEN", token.c_str(), 1);
    web_ = Process::spawn({executable(root_, "shaoshao_web", "shaoshao_web").string(),
                           "--socket", (root_ / "web-supervisor.sock").string(),
                           "--assets", assets.string(), "--maps-root", manager_.maps_base.string(),
                           "--bind", "0.0.0.0", "--port", "8080"}, root_ / "web.log");
    atomic_write(root_ / "web.pid", std::to_string(web_.pid()) + "\n");
  }

  void refresh() {
    std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
    tasks_.maps = manager_.maps;
    tasks_.mode = manager_.mode();
    tasks_.mapping_saved = manager_.saved;
    tasks_.stack_ready = manager_.navigation_running() || manager_.mapping_running();
    tasks_.mapping_saved = manager_.saved;
    tasks_.tick(steady());
  }

  void request_mode(const std::string& mode) {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    if (transition_running_) throw std::runtime_error("正在切换模式，请稍候");
    transition_running_ = true;
    transition_ = std::jthread([this, mode](std::stop_token stop) {
      try {
        if (!stop.stop_requested()) {
          manager_.transition(mode);
          tasks_.mode_finished(manager_.mode(), manager_.maps);
        }
      } catch (const std::exception& e) {
        tasks_.mode_finished(manager_.mode(), manager_.maps, e.what());
      }
      transition_running_ = false;
    });
  }

  void trigger(const std::string& name) {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    if (name == "/map_save") client = save_map_;
    else if (name == "/hw/finalize_index") client = finalize_index_;
    else if (name == "/hw/localization/retry") client = retry_localization_;
    else throw std::invalid_argument("Unknown ROS trigger service");
    if (!client->wait_for_service(std::chrono::seconds(2)))
      throw std::runtime_error("ROS service unavailable: " + name);
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
      throw std::runtime_error("ROS service timeout: " + name);
    auto response = future.get();
    if (!response->success) throw std::runtime_error(response->message);
  }
};

} // namespace hw_runtime

int main(int argc, char** argv) {
  using namespace hw_runtime;
  try {
    fs::path root = runtime_path(), maps;
    for (int i = 1; i < argc; ++i) {
      std::string option = argv[i];
      if (i + 1 >= argc) throw std::invalid_argument("Missing supervisor argument");
      auto value = fs::path(argv[++i]);
      if (option == "--runtime") root = value;
      else if (option == "--maps-dir") maps = value;
      else throw std::invalid_argument("Unknown supervisor argument");
    }
    if (maps.empty()) maps = root / "maps";
    rclcpp::init(argc, argv);
    hw_runtime::Supervisor supervisor(root, maps);
    supervisor.run();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Native supervisor failed: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
