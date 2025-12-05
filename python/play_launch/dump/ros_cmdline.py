from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class CommandLine:
    command: str
    user_args: List[str]
    remaps: Dict[str, str]
    params: Dict[str, str]
    params_files: List[str]
    log_level: Optional[str]
    log_config_file: Optional[str]
    enable_rosout_logs: Optional[bool]
    enable_stdout_logs: Optional[bool]
    enclave: Optional[str]


def parse_ros_cmdline(cmdline: List[str]):
    command = cmdline[0]
    args = cmdline[1:]

    remain_args = args
    user_args = list()
    ros_args = list()

    while True:

        def find(iterable, target):
            return next((ix for ix, val in enumerate(iterable) if val == target), None)

        ix = find(remain_args, "--ros-args")
        if ix is None:
            user_args.extend(remain_args)
            break

        user_args.extend(remain_args[:ix])
        remain_args = remain_args[(ix + 1) :]

        ix = find(remain_args, "--")
        if ix is None:
            ros_args.extend(filter(lambda arg: arg != "--ros-args", remain_args))
            break
        ros_args.extend(filter(lambda arg: arg != "--ros-args", remain_args[:ix]))
        remain_args = remain_args[(ix + 1) :]

    remaps: Dict[str, str] = dict()
    params: Dict[str, str] = dict()
    params_files: List[str] = list()
    log_level: Optional[str] = None
    log_config_file: Optional[str] = None
    enable_rosout_logs: Optional[bool] = None
    enable_stdout_logs: Optional[bool] = None
    enclave: Optional[str] = None

    ros_args_iter = iter(ros_args)

    while True:
        try:
            arg = next(ros_args_iter)
        except StopIteration:
            break

        if arg in ["-r", "--remap"]:
            expr = next(ros_args_iter, None)
            assert expr is not None
            name, value = expr.split(":=")
            remaps[name] = value

        elif arg in ["-p", "--param"]:
            expr = next(ros_args_iter, None)
            assert expr is not None
            name, value = expr.split(":=")
            params[name] = value

        elif arg == "--params-file":
            path = next(ros_args_iter, None)
            assert path is not None
            params_files.append(path)

        elif arg == "--log-level":
            level = next(ros_args_iter, None)
            assert level is not None
            log_level = level

        elif arg == "--log-config-file":
            path = next(ros_args_iter, None)
            assert path is not None
            log_config_file = path

        elif arg == "--enable-rosout-logs":
            enable_rosout_logs = True

        elif arg == "--disable-rosout-logs":
            enable_rosout_logs = False

        elif arg == "--enable-stdout-logs":
            enable_stdout_logs = True

        elif arg == "--disable-stdout-logs":
            enable_stdout_logs = False

        elif arg in ["-e", "--enclave"]:
            value = next(ros_args_iter, None)
            assert value is not None
            enclave = value

        else:
            raise ValueError(f"unknown argument {arg}")

    return CommandLine(
        command=command,
        user_args=user_args,
        remaps=remaps,
        params=params,
        params_files=params_files,
        log_level=log_level,
        log_config_file=log_config_file,
        enable_rosout_logs=enable_rosout_logs,
        enable_stdout_logs=enable_stdout_logs,
        enclave=enclave,
    )
