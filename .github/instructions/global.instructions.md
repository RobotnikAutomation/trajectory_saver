---
name: ROS 2 Package Development
description: "Use when creating or modifying ROS 2 packages, nodes, launch files, YAML configuration, ROS interfaces, or package tests. Covers package layout, interface naming, C++ node style, configuration, and validation."
applyTo: "**"
---

# ROS 2 Package Development Guidelines

## Package Structure

- For every C++ node, use exactly two `.cpp` files and one header: a node entry-point file containing `main()`, a separate implementation file containing the node behavior, and a declaration-only header containing the class declarations and members.
- Keep `main()` focused on initialization, node startup, and executor spinning. Keep ROS interface setup, parameters, callbacks, and runtime behavior in the implementation `.cpp` file.
- Provide a corresponding YAML parameter file for each node and load it from that node's launch file. Apply launch arguments only as explicit overrides to the YAML defaults.
- Keep tests with the package. Update its README when adding or changing public parameters, interfaces, launch behavior, or workflows.
- Declare runtime and build dependencies in the package manifest and build configuration. Do not rely on dependencies being present transitively.

## Package `.gitignore`

- When creating a package, ensure its root contains a `.gitignore`. Create one with the canonical rules below if it is missing.
- For an existing `.gitignore`, preserve package-specific rules, append any missing canonical rules, and remove duplicate identical entries while keeping one copy.
- Do not replace an existing `.gitignore` with only the canonical rules.

```gitignore
.vscode/
.tmp/
*.png
*.pcd
*.ply
*.pyc
*.pswp
!.gitignore
__pycache__/
.coverage
.agent/
.codex/
.obj-x86_64-linux-gnu/
debian/
*.deb
*.ddeb
```

## ROS Interface Design

- Make names of interfaces a node consumes configurable parameters, including subscribed topics and client service/action names.
- Keep interface-name defaults in descriptively named header members such as `joint_states_topic_name_`, `pose_service_name_`, and `trajectory_action_name_`; read input-name parameters in the implementation using matching parameter keys without the trailing underscore.
- Define fully qualified hosted service/action names and published topic names under the required robot namespace in descriptively named header members. Never use `~/` or relative names for ROS interfaces. Do not expose output names as YAML parameters or launch overrides.
- Use named member callbacks registered with `std::bind` where callbacks operate on node state.
- Keep service clients and service servers distinct: clients call remote endpoints; servers implement endpoints owned by the node.
- Prefer concrete, typed overloads to generic templates when a helper serves a small, fixed set of ROS interface types.
- Preserve each interface's ownership boundary. A coordinator should call the responsible storage, planning, or driver package instead of reaching into its implementation details.

## C++ Style

- Use descriptive names for parameters, members, callbacks, requests, responses, and intermediate values.
- Use explicit `this->` for inherited `rclcpp::Node` API calls and callback binding. Do not add `this->` to ordinary member-field or helper-method access.
- Keep headers declarative. Avoid callback lambdas or nontrivial method implementations in headers unless a template requires its definition there.
- Add a concise purpose comment to each nontrivial method and callback. Use inline comments to explain non-obvious data flow, invariants, ordering, retries, cleanup, and persistence guarantees. Keep comments accurate as code changes; do not narrate obvious statements.
- After each code change, make the next step a review of comments in the changed code; revise or add comments so they explain the updated behavior and remain accurate. After that review, check the package README and update it when the change adds a feature or changes documented behavior, interfaces, parameters, launch instructions, or workflows.
- Validate parameters before creating interfaces. Use bounded waits for external services and avoid blocking an executor in a way that prevents callbacks needed to complete the operation.

## Behavior and Tests

- Identify and preserve the feature's invariants before changing its implementation. If one operation must collect or update several related representations, keep them consistent and test each representation independently.
- Test invalid inputs, unavailable dependencies, timeouts, and failure responses as well as successful behavior.
- Use mocks or temporary resources for integration tests; do not require a physical robot or modify operational data unless explicitly requested.
- Test interactions through public ROS interfaces where practical, including the configured names, message/service types, and launch-provided parameters.

## Build and Validation

- Source the intended ROS 2 distribution before building or testing, and use the workspace's documented build commands.
- In workspaces containing multiple ROS distributions or duplicate package names, pass explicit colcon base paths and package selections to avoid building the wrong package.
- After changing code, run the focused build and tests for the affected package and its direct integration tests. Validate launch and YAML changes with the corresponding launch description.
- Update documentation and examples when public configuration or interface behavior changes; do not document internal implementation details as public APIs.