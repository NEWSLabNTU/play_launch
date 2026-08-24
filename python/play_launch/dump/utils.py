import decimal

import yaml
from rcl_interfaces.msg import Parameter

DECIMAL_CONTEXT = decimal.Context()
DECIMAL_CONTEXT.prec = 20


def float_to_str(f):
    """
    Convert the given float to a string,
    without resorting to scientific notation
    """
    global DECIMAL_CONTEXT
    d = DECIMAL_CONTEXT.create_decimal(repr(f))
    return format(d, "f")


def log_level_code_to_text(code: int) -> str | None:
    match code:
        case 0:
            return None
        case 10:
            return "DEBUG"
        case 20:
            return "INFO"
        case 30:
            return "WARN"
        case 40:
            return "ERROR"
        case 50:
            return "FATAL"
        case _:
            raise ValueError(f"unknown log level code {code}")


def text_to_kv(expr: str):
    return tuple(expr.split(":=", 1))


def param_to_kv(param: Parameter) -> tuple[str, str]:
    pvalue = param.value

    match pvalue.type:
        case 1:
            value = dump_yaml(pvalue.bool_value)
        case 2:
            value = dump_yaml(pvalue.integer_value)
        case 3:
            # NOTE: ruamle.yaml serializes small floats into scientific
            # notations like 1e-7. It is treated as a string by `ros2`
            # command. It uses an alternative way to work around.

            # value = dump_yaml(pvalue.double_value)
            value = float_to_str(pvalue.double_value)
        case 4:
            value = dump_yaml(pvalue.string_value)
        case 5:
            value = dump_yaml(list(pvalue.byte_array_value))
        case 6:
            value = dump_yaml(list(pvalue.bool_array_value))
        case 7:
            value = dump_yaml(list(pvalue.integer_array_value))
        case 8:
            # NOTE: ruamle.yaml serializes small floats into scientific
            # notations like 1e-7. It is treated as a string by `ros2`
            # command. It uses an alternative way to work around.

            # value = dump_yaml(list(pvalue.double_array_value))
            value = "[" + ", ".join(float_to_str(f) for f in pvalue.double_array_value) + "]"
        case 9:
            value = dump_yaml(list(pvalue.string_array_value))
        case _:
            raise ValueError(f"unknown parameter type code {param.type}")

    return param.name, value


def dump_yaml(value) -> str:
    result = yaml.dump(
        value,
        default_flow_style=True,
        width=65536,
        allow_unicode=True,
    )
    # Strip trailing newline and "...\n" if present
    result = result.rstrip("\n")
    if result.endswith("..."):
        result = result[:-3].rstrip()
    return result

def param_value_to_str(value) -> str:
    """Render a parameter value for the spawned node.

    Strings pass through verbatim. They are already the exact text the node expects, and
    YAML-encoding them wraps them in quotes and escapes their newlines -- which silently
    destroyed `robot_description`, whose value is a whole URDF document. robot_state_publisher
    then aborted with "Error document empty", nothing published /robot_description, and
    acb_bridge waited for it forever.

    Everything else goes through YAML, because Python's repr is not what a node parses:
    str(['camera6']) is "['camera6']" and the quotes end up inside the value, and str(True)
    is "True" where ROS wants "true".
    """
    if isinstance(value, str):
        return value
    return dump_yaml(value)
