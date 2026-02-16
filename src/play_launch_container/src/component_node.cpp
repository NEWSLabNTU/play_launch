// Copyright 2026 play_launch developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Standalone binary that loads and spins a single composable node.
///
/// Usage:
///   component_node \
///     --package composition --plugin composition::Talker \
///     --ready-fd 5 [--use-multi-threaded-executor] \
///     --ros-args -r __node:=talker -r __ns:=/ --params-file /tmp/params.yaml
///
/// The parent (CloneIsolatedComponentManager) forks and execs this binary for
/// each composable node.  The child writes "OK <full_node_name>\n" to the
/// ready-fd pipe on success, or "ERR <message>\n" on failure, then closes
/// the pipe and spins the executor until SIGTERM.

#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

static void write_ready(int fd, const std::string & msg)
{
  std::string line = msg + "\n";
  size_t written = 0;
  while (written < line.size()) {
    auto n = write(fd, line.data() + written, line.size() - written);
    if (n <= 0) {
      break;
    }
    written += static_cast<size_t>(n);
  }
  close(fd);
}

int main(int argc, char * argv[])
{
  // Parse our custom args (before --ros-args).  Everything after --ros-args
  // is passed to rclcpp::init().
  std::string package_name;
  std::string plugin_name;
  int ready_fd = -1;
  bool use_multi_threaded = false;
  bool use_intra_process_comms = false;

  int ros_args_start = argc;  // index of --ros-args in argv
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--ros-args") {
      ros_args_start = i;
      break;
    }
    if (arg == "--package" && i + 1 < argc) {
      package_name = argv[++i];
    } else if (arg == "--plugin" && i + 1 < argc) {
      plugin_name = argv[++i];
    } else if (arg == "--ready-fd" && i + 1 < argc) {
      ready_fd = std::atoi(argv[++i]);
    } else if (arg == "--use-multi-threaded-executor") {
      use_multi_threaded = true;
    } else if (arg == "--use-intra-process-comms") {
      use_intra_process_comms = true;
    }
  }

  if (package_name.empty() || plugin_name.empty()) {
    const char * msg = "component_node: --package and --plugin are required\n";
    if (write(STDERR_FILENO, msg, std::strlen(msg)) < 0) {
    }
    if (ready_fd >= 0) {
      write_ready(ready_fd, "ERR --package and --plugin are required");
    }
    return 1;
  }

  // Build argc/argv for rclcpp::init from --ros-args onward
  // We include argv[0] (program name) plus everything from --ros-args
  int ros_argc = 1 + (argc - ros_args_start);
  std::vector<char *> ros_argv;
  ros_argv.push_back(argv[0]);
  for (int i = ros_args_start; i < argc; ++i) {
    ros_argv.push_back(argv[i]);
  }

  rclcpp::init(ros_argc, ros_argv.data());

  // Find the plugin shared library via ament index
  std::string content;
  std::string prefix;
  if (!ament_index_cpp::get_resource("rclcpp_components", package_name, content, &prefix)) {
    std::string err = "No rclcpp_components index for package '" + package_name + "'";
    if (ready_fd >= 0) {
      write_ready(ready_fd, "ERR " + err);
    }
    rclcpp::shutdown();
    return 1;
  }

  // Parse ament resource lines: "plugin::Name;lib/libfoo.so\n..."
  // Format: class_name;relative_library_path  (class_name is BEFORE semicolon)
  std::string library_path;
  std::istringstream stream(content);
  std::string line;
  while (std::getline(stream, line)) {
    auto sep = line.find(';');
    if (sep == std::string::npos) {
      continue;
    }
    std::string registered_plugin = line.substr(0, sep);
    std::string lib_rel = line.substr(sep + 1);
    // Trim trailing whitespace/newline
    while (!lib_rel.empty() &&
           (lib_rel.back() == '\n' || lib_rel.back() == '\r' || lib_rel.back() == ' ')) {
      lib_rel.pop_back();
    }
    if (registered_plugin == plugin_name) {
      library_path = prefix + "/" + lib_rel;
      break;
    }
  }

  if (library_path.empty()) {
    std::string err = "Plugin '" + plugin_name + "' not found in package '" + package_name + "'";
    if (ready_fd >= 0) {
      write_ready(ready_fd, "ERR " + err);
    }
    rclcpp::shutdown();
    return 1;
  }

  // Load the plugin library and create the node
  try {
    class_loader::ClassLoader loader(library_path);
    auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();

    // Match by bare name or template-wrapped name (same logic as
    // rclcpp_components::ComponentManager::create_component_factory)
    std::string wrapped_name = "rclcpp_components::NodeFactoryTemplate<" + plugin_name + ">";
    std::shared_ptr<rclcpp_components::NodeFactory> factory;
    for (const auto & cls : classes) {
      if (cls == plugin_name || cls == wrapped_name) {
        factory = loader.createInstance<rclcpp_components::NodeFactory>(cls);
        break;
      }
    }

    if (!factory) {
      std::string err = "Failed to create NodeFactory for '" + plugin_name + "'";
      if (ready_fd >= 0) {
        write_ready(ready_fd, "ERR " + err);
      }
      rclcpp::shutdown();
      return 1;
    }

    // rclcpp::init() already parsed --ros-args (remappings, params-file)
    // so NodeOptions() picks them up via default context
    rclcpp::NodeOptions options;
    options.use_global_arguments(true);
    options.use_intra_process_comms(use_intra_process_comms);
    auto wrapper = factory->create_node_instance(options);
    auto node_base = wrapper.get_node_base_interface();
    std::string full_name = node_base->get_fully_qualified_name();

    // Signal success to parent
    if (ready_fd >= 0) {
      write_ready(ready_fd, "OK " + full_name);
    }

    // Spin until SIGTERM
    std::shared_ptr<rclcpp::Executor> exec;
    if (use_multi_threaded) {
      exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    } else {
      exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }
    exec->add_node(node_base);
    exec->spin();

    exec->remove_node(node_base);
  } catch (const std::exception & ex) {
    std::string err = "Exception loading '" + plugin_name + "': " + ex.what();
    if (ready_fd >= 0) {
      write_ready(ready_fd, "ERR " + err);
    }
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
