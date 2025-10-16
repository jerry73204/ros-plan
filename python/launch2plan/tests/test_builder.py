"""
Tests for plan builder module.

Phase 4: Plan Builder & Link Generation
"""

import pytest

from launch2plan.builder import PlanBuilder, PlanLink
from launch2plan.inference import InferredSocket
from launch2plan.visitor import LaunchArgumentMetadata, NodeMetadata


@pytest.fixture
def plan_builder():
    """Create a plan builder instance."""
    return PlanBuilder()


def test_build_node_section(plan_builder):
    """Test node section generation."""
    # Setup: Simple node with sockets
    nodes = [
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="talker",
            name="talker_node",
            remappings=[("chatter", "/demo/chatter")],
        )
    ]

    inferred_sockets = {
        "talker_node": [
            InferredSocket(
                socket_name="chatter",
                direction="!pub",
                message_type="std_msgs/msg/String",
                remapped_to="/demo/chatter",
                source="introspection",
            )
        ]
    }

    # Execute: Build node section
    node_section = plan_builder._build_node_section(nodes, inferred_sockets)

    # Verify: Node structure
    assert "talker_node" in node_section
    node = node_section["talker_node"]
    assert node["pkg"] == "demo_nodes_cpp"
    assert node["exec"] == "talker"
    assert "socket" in node
    assert node["socket"]["chatter"] == "!pub"


def test_build_socket_section(plan_builder):
    """Test socket section generation with TODO markers."""
    # Setup: Mix of resolved and TODO sockets
    sockets = [
        InferredSocket(
            socket_name="output",
            direction="!pub",
            message_type="std_msgs/msg/String",
            remapped_to="/topic1",
            source="introspection",
        ),
        InferredSocket(
            socket_name="input",
            direction=None,  # TODO
            message_type=None,
            remapped_to="/topic2",
            comment="TODO: Specify direction",
            source="todo",
        ),
    ]

    # Execute: Build socket section
    socket_section = plan_builder._build_socket_section(sockets, "test_node")

    # Verify: Both sockets present
    assert socket_section["output"] == "!pub"
    assert socket_section["input"] == "!todo"


def test_infer_links_from_remappings(plan_builder):
    """Test link inference by matching remappings across nodes."""
    # Setup: Talker and listener nodes remapping to same topic
    nodes = [
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="talker",
            name="talker",
            remappings=[("chatter", "/demo/chatter")],
        ),
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="listener",
            name="listener",
            remappings=[("chatter", "/demo/chatter")],
        ),
    ]

    inferred_sockets = {
        "talker": [
            InferredSocket(
                socket_name="chatter",
                direction="!pub",
                message_type="std_msgs/msg/String",
                remapped_to="/demo/chatter",
                source="introspection",
            )
        ],
        "listener": [
            InferredSocket(
                socket_name="chatter",
                direction="!sub",
                message_type="std_msgs/msg/String",
                remapped_to="/demo/chatter",
                source="introspection",
            )
        ],
    }

    # Execute: Infer links
    links = plan_builder._infer_links(nodes, inferred_sockets)

    # Verify: One link created
    assert len(links) == 1
    link = links[0]
    assert link.name == "demo_chatter"
    assert link.message_type == "std_msgs/msg/String"
    assert "talker/chatter" in link.sources
    assert "listener/chatter" in link.destinations


def test_generate_link_section(plan_builder):
    """Test link section YAML generation."""
    # Setup: Link with complete information
    links = [
        PlanLink(
            name="camera_image",
            message_type="sensor_msgs/msg/Image",
            sources=["camera/image_out"],
            destinations=["processor/image_in", "logger/image_in"],
        )
    ]

    # Execute: Build link section
    link_section = plan_builder._build_link_section(links)

    # Verify: Link structure
    assert "camera_image" in link_section
    link_def = link_section["camera_image"]["!pubsub"]
    assert link_def["type"] == "sensor_msgs/msg/Image"
    assert link_def["src"] == ["camera/image_out"]
    assert link_def["dst"] == ["processor/image_in", "logger/image_in"]


def test_full_plan_generation(plan_builder):
    """Test complete plan generation with nodes and links."""
    # Setup: Complete talker/listener setup
    nodes = [
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="talker",
            name="talker",
            namespace="/demo",
            parameters=[{"use_sim_time": False}],
            remappings=[("chatter", "/demo/chatter")],
        ),
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="listener",
            name="listener",
            namespace="/demo",
            remappings=[("chatter", "/demo/chatter")],
        ),
    ]

    inferred_sockets = {
        "talker": [
            InferredSocket(
                socket_name="chatter",
                direction="!pub",
                message_type="std_msgs/msg/String",
                remapped_to="/demo/chatter",
                source="introspection",
            )
        ],
        "listener": [
            InferredSocket(
                socket_name="chatter",
                direction="!sub",
                message_type="std_msgs/msg/String",
                remapped_to="/demo/chatter",
                source="introspection",
            )
        ],
    }

    # Execute: Build complete plan
    plan = plan_builder.build_plan(nodes, inferred_sockets)

    # Verify: Plan has both sections
    assert "node" in plan
    assert "link" in plan

    # Verify nodes
    assert "talker" in plan["node"]
    assert "listener" in plan["node"]
    assert plan["node"]["talker"]["pkg"] == "demo_nodes_cpp"
    assert plan["node"]["talker"]["ns"] == "/demo"
    assert plan["node"]["talker"]["socket"]["chatter"] == "!pub"

    # Verify links
    assert "demo_chatter" in plan["link"]
    link_def = plan["link"]["demo_chatter"]["!pubsub"]
    assert link_def["type"] == "std_msgs/msg/String"
    assert link_def["src"] == ["talker/chatter"]
    assert link_def["dst"] == ["listener/chatter"]


def test_plan_with_todo_sockets(plan_builder):
    """Test plan generation with TODO markers for missing information."""
    # Setup: Node with unknown socket direction
    nodes = [
        NodeMetadata(
            package="custom_pkg",
            executable="custom_node",
            name="custom",
            remappings=[("data", "/data/topic")],
        )
    ]

    inferred_sockets = {
        "custom": [
            InferredSocket(
                socket_name="data",
                direction=None,  # Unknown
                message_type=None,
                remapped_to="/data/topic",
                comment="TODO: Specify direction",
                source="todo",
            )
        ]
    }

    # Execute: Build plan
    plan = plan_builder.build_plan(nodes, inferred_sockets)

    # Verify: Node has TODO socket
    assert plan["node"]["custom"]["socket"]["data"] == "!todo"

    # Verify: No links created (no direction known)
    assert "link" not in plan or len(plan["link"]) == 0


def test_link_with_todo_message_type(plan_builder):
    """Test link generation when message type is unknown."""
    # Setup: Nodes with sockets but unknown message type
    nodes = [
        NodeMetadata(
            package="pkg1",
            executable="node1",
            name="pub_node",
            remappings=[("out", "/topic")],
        ),
        NodeMetadata(
            package="pkg2",
            executable="node2",
            name="sub_node",
            remappings=[("in", "/topic")],
        ),
    ]

    inferred_sockets = {
        "pub_node": [
            InferredSocket(
                socket_name="out",
                direction="!pub",
                message_type=None,  # Unknown
                remapped_to="/topic",
                source="introspection",
            )
        ],
        "sub_node": [
            InferredSocket(
                socket_name="in",
                direction="!sub",
                message_type=None,  # Unknown
                remapped_to="/topic",
                source="introspection",
            )
        ],
    }

    # Execute: Build plan
    plan = plan_builder.build_plan(nodes, inferred_sockets)

    # Verify: Link created with TODO type
    assert "link" in plan
    assert "topic" in plan["link"]
    link_def = plan["link"]["topic"]["!pubsub"]
    assert link_def["type"] == "TODO"


def test_multiple_publishers_and_subscribers(plan_builder):
    """Test link generation with multiple publishers and subscribers."""
    # Setup: Multiple cameras publishing, multiple processors subscribing
    nodes = [
        NodeMetadata(
            package="camera_pkg",
            executable="camera",
            name="camera1",
            remappings=[("image", "/images")],
        ),
        NodeMetadata(
            package="camera_pkg",
            executable="camera",
            name="camera2",
            remappings=[("image", "/images")],
        ),
        NodeMetadata(
            package="proc_pkg",
            executable="processor",
            name="proc1",
            remappings=[("input", "/images")],
        ),
        NodeMetadata(
            package="proc_pkg",
            executable="processor",
            name="proc2",
            remappings=[("input", "/images")],
        ),
    ]

    inferred_sockets = {
        "camera1": [
            InferredSocket(
                socket_name="image",
                direction="!pub",
                message_type="sensor_msgs/msg/Image",
                remapped_to="/images",
                source="introspection",
            )
        ],
        "camera2": [
            InferredSocket(
                socket_name="image",
                direction="!pub",
                message_type="sensor_msgs/msg/Image",
                remapped_to="/images",
                source="introspection",
            )
        ],
        "proc1": [
            InferredSocket(
                socket_name="input",
                direction="!sub",
                message_type="sensor_msgs/msg/Image",
                remapped_to="/images",
                source="introspection",
            )
        ],
        "proc2": [
            InferredSocket(
                socket_name="input",
                direction="!sub",
                message_type="sensor_msgs/msg/Image",
                remapped_to="/images",
                source="introspection",
            )
        ],
    }

    # Execute: Build plan
    plan = plan_builder.build_plan(nodes, inferred_sockets)

    # Verify: Single link with multiple sources and destinations
    assert "link" in plan
    assert "images" in plan["link"]
    link_def = plan["link"]["images"]["!pubsub"]
    assert len(link_def["src"]) == 2
    assert len(link_def["dst"]) == 2
    assert "camera1/image" in link_def["src"]
    assert "camera2/image" in link_def["src"]
    assert "proc1/input" in link_def["dst"]
    assert "proc2/input" in link_def["dst"]


def test_conditional_node(plan_builder):
    """Test node with when clause."""
    # Setup: Node with condition
    nodes = [
        NodeMetadata(
            package="debug_pkg",
            executable="debugger",
            name="debug_node",
            condition_expr="$(debug_mode)",
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan(nodes, {})

    # Verify: When clause included
    assert plan["node"]["debug_node"]["when"] == "$(debug_mode)"


def test_plan_to_yaml_string(plan_builder):
    """Test YAML string generation."""
    # Setup: Simple plan
    nodes = [
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="talker",
            name="talker",
        )
    ]

    plan = plan_builder.build_plan(nodes, {})

    # Execute: Convert to YAML string
    yaml_string = plan_builder.plan_to_yaml_string(plan)

    # Verify: YAML is valid and contains expected content
    assert "node:" in yaml_string
    assert "talker:" in yaml_string
    assert "pkg: demo_nodes_cpp" in yaml_string
    assert "exec: talker" in yaml_string


# Phase 5: Argument & Parameter Conversion Tests


def test_build_arg_section_with_bool(plan_builder):
    """Test arg section generation with boolean argument."""
    # Setup: Boolean argument
    launch_args = [
        LaunchArgumentMetadata(
            name="use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan([], {}, launch_args)

    # Verify: Arg section with !bool tag
    assert "arg" in plan
    assert "use_sim_time" in plan["arg"]
    assert plan["arg"]["use_sim_time"]["!bool"] is True


def test_build_arg_section_with_int(plan_builder):
    """Test arg section generation with integer argument."""
    # Setup: Integer argument
    launch_args = [
        LaunchArgumentMetadata(
            name="port",
            default_value="8080",
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan([], {}, launch_args)

    # Verify: Arg section with !i64 tag
    assert "arg" in plan
    assert "port" in plan["arg"]
    assert plan["arg"]["port"]["!i64"] == 8080


def test_build_arg_section_with_float(plan_builder):
    """Test arg section generation with float argument."""
    # Setup: Float argument
    launch_args = [
        LaunchArgumentMetadata(
            name="rate",
            default_value="10.5",
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan([], {}, launch_args)

    # Verify: Arg section with !f64 tag
    assert "arg" in plan
    assert "rate" in plan["arg"]
    assert plan["arg"]["rate"]["!f64"] == 10.5


def test_build_arg_section_with_string(plan_builder):
    """Test arg section generation with string argument."""
    # Setup: String argument
    launch_args = [
        LaunchArgumentMetadata(
            name="robot_name",
            default_value="robot1",
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan([], {}, launch_args)

    # Verify: Arg section with !str tag
    assert "arg" in plan
    assert "robot_name" in plan["arg"]
    assert plan["arg"]["robot_name"]["!str"] == "robot1"


def test_build_arg_section_with_no_default(plan_builder):
    """Test arg section generation when no default value provided."""
    # Setup: Argument with no default
    launch_args = [
        LaunchArgumentMetadata(
            name="required_arg",
            default_value=None,
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan([], {}, launch_args)

    # Verify: Arg section with !todo tag
    assert "arg" in plan
    assert "required_arg" in plan["arg"]
    assert plan["arg"]["required_arg"]["!todo"] is None


def test_build_arg_section_with_multiple_types(plan_builder):
    """Test arg section generation with multiple argument types."""
    # Setup: Mixed argument types
    launch_args = [
        LaunchArgumentMetadata(name="use_sim_time", default_value="false"),
        LaunchArgumentMetadata(name="port", default_value="9000"),
        LaunchArgumentMetadata(name="rate", default_value="20.0"),
        LaunchArgumentMetadata(name="namespace", default_value="/robot"),
        LaunchArgumentMetadata(name="required", default_value=None),
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan([], {}, launch_args)

    # Verify: All arguments present with correct types
    assert "arg" in plan
    assert plan["arg"]["use_sim_time"]["!bool"] is False
    assert plan["arg"]["port"]["!i64"] == 9000
    assert plan["arg"]["rate"]["!f64"] == 20.0
    assert plan["arg"]["namespace"]["!str"] == "/robot"
    assert plan["arg"]["required"]["!todo"] is None


def test_launch_configuration_substitution_in_params(plan_builder):
    """Test LaunchConfiguration substitution in node parameters."""
    from launch.substitutions import LaunchConfiguration

    # Setup: Node with LaunchConfiguration in parameters
    nodes = [
        NodeMetadata(
            package="test_pkg",
            executable="test_node",
            name="test_node",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan(nodes, {})

    # Verify: LaunchConfiguration converted to $(arg_name)
    assert plan["node"]["test_node"]["param"]["use_sim_time"] == "$(use_sim_time)"


def test_nested_launch_configuration_in_params(plan_builder):
    """Test nested LaunchConfiguration in node parameters."""
    from launch.substitutions import LaunchConfiguration

    # Setup: Node with nested parameters containing LaunchConfiguration
    nodes = [
        NodeMetadata(
            package="test_pkg",
            executable="test_node",
            name="test_node",
            parameters=[
                {
                    "config": {
                        "rate": LaunchConfiguration("publish_rate"),
                        "enabled": True,
                    }
                }
            ],
        )
    ]

    # Execute: Build plan
    plan = plan_builder.build_plan(nodes, {})

    # Verify: Nested LaunchConfiguration converted
    assert plan["node"]["test_node"]["param"]["config"]["rate"] == "$(publish_rate)"
    assert plan["node"]["test_node"]["param"]["config"]["enabled"] is True


def test_complete_plan_with_args_and_params(plan_builder):
    """Test complete plan generation with arguments and parameter substitutions."""
    from launch.substitutions import LaunchConfiguration

    # Setup: Launch arguments and nodes with parameter substitutions
    launch_args = [
        LaunchArgumentMetadata(name="use_sim_time", default_value="false"),
        LaunchArgumentMetadata(name="rate", default_value="10.0"),
    ]

    nodes = [
        NodeMetadata(
            package="test_pkg",
            executable="publisher",
            name="publisher",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_rate": LaunchConfiguration("rate"),
                }
            ],
            remappings=[("output", "/data")],
        )
    ]

    inferred_sockets = {
        "publisher": [
            InferredSocket(
                socket_name="output",
                direction="!pub",
                message_type="std_msgs/msg/String",
                remapped_to="/data",
                source="introspection",
            )
        ]
    }

    # Execute: Build complete plan
    plan = plan_builder.build_plan(nodes, inferred_sockets, launch_args)

    # Verify: Args section
    assert "arg" in plan
    assert plan["arg"]["use_sim_time"]["!bool"] is False
    assert plan["arg"]["rate"]["!f64"] == 10.0

    # Verify: Node with converted parameters
    assert "node" in plan
    assert plan["node"]["publisher"]["param"]["use_sim_time"] == "$(use_sim_time)"
    assert plan["node"]["publisher"]["param"]["publish_rate"] == "$(rate)"

    # Verify: Socket section
    assert plan["node"]["publisher"]["socket"]["output"] == "!pub"
