"""Tests for parse_file()."""

import json

import pytest

from play_launch_parser import parse_file


class TestReturnStructure:
    """Verify the shape of the returned dict."""

    def test_top_level_keys(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        assert isinstance(result, dict)
        for key in ("node", "container", "load_node", "scopes", "file_data", "lifecycle_node"):
            assert key in result, f"Missing key: {key}"

    def test_node_is_list_of_dicts(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        assert isinstance(result["node"], list)
        assert len(result["node"]) > 0
        assert isinstance(result["node"][0], dict)

    def test_node_fields(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        node = result["node"][0]
        # Required fields
        assert "cmd" in node
        assert "executable" in node
        assert isinstance(node["cmd"], list)
        assert all(isinstance(s, str) for s in node["cmd"])
        # Optional fields present as None or value
        for field in ("name", "namespace", "package", "exec_name"):
            assert field in node

    def test_json_roundtrip(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        text = json.dumps(result)
        roundtripped = json.loads(text)
        assert roundtripped == result


class TestXmlParsing:
    """Test XML launch file parsing."""

    def test_args_defaults(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        assert len(result["node"]) == 1
        node = result["node"][0]
        assert node["name"] == "my_talker"
        assert node["package"] == "demo_nodes_cpp"
        assert node["executable"] == "talker"

    def test_args_override(self, fixture):
        result = parse_file(
            fixture("test_args.launch.xml"),
            args={"node_name": "custom_name"},
        )
        node = result["node"][0]
        assert node["name"] == "custom_name"

    def test_params(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        node = result["node"][0]
        param_keys = [p[0] for p in node["params"]]
        assert "use_sim_time" in param_keys

    def test_remaps(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        node = result["node"][0]
        assert ["chatter", "/my_topic"] in node["remaps"]

    def test_env(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        node = result["node"][0]
        assert node["env"] is not None
        env_keys = [e[0] for e in node["env"]]
        assert "MY_ENV_VAR" in env_keys

    def test_conditions_if_true(self, fixture):
        result = parse_file(fixture("test_conditions.launch.xml"))
        names = [n["name"] for n in result["node"]]
        assert "sim_talker" in names  # if="true"
        assert "rviz_listener" not in names  # if="false"

    def test_conditions_unless(self, fixture):
        result = parse_file(fixture("test_conditions.launch.xml"))
        names = [n["name"] for n in result["node"]]
        assert "normal_listener" in names  # unless="false" → included
        assert "real_talker" not in names  # unless="true" → excluded

    def test_conditions_override(self, fixture):
        result = parse_file(
            fixture("test_conditions.launch.xml"),
            args={"use_rviz": "true"},
        )
        names = [n["name"] for n in result["node"]]
        assert "rviz_listener" in names
        assert "normal_listener" not in names


class TestScopes:
    """Test launch tree scope tracking."""

    def test_scopes_present(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        assert len(result["scopes"]) >= 1

    def test_root_scope(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        root = result["scopes"][0]
        assert root["id"] == 0
        assert root["parent"] is None
        assert "origin" in root

    def test_node_scope_reference(self, fixture):
        result = parse_file(fixture("test_args.launch.xml"))
        node = result["node"][0]
        if node.get("scope") is not None:
            assert node["scope"] < len(result["scopes"])


class TestErrors:
    """Test error handling."""

    def test_file_not_found(self):
        with pytest.raises(FileNotFoundError):
            parse_file("/nonexistent/path.launch.xml")

    def test_invalid_xml(self, tmp_path):
        bad_file = tmp_path / "bad.launch.xml"
        bad_file.write_text("<launch><node></launch>")
        with pytest.raises(RuntimeError):
            parse_file(str(bad_file))

    def test_empty_args(self, fixture):
        # Should work fine with empty args dict
        result = parse_file(fixture("test_args.launch.xml"), args={})
        assert len(result["node"]) == 1
