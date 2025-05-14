#!/usr/bin/env python3
import argparse
import os
import subprocess

INFO_COLOUR = "\033[93m"
RESET_COLOUR = "\033[0m"

PACKAGE_PATH = f"{os.getcwd()}/src"
CV_PATH = PACKAGE_PATH + "/computer_vision"
CV_CONFIG_PATH = CV_PATH + "/config"
CV_TRAIN_PATH = CV_PATH + "/train"


def info(to_print):
    print(f"{INFO_COLOUR}{to_print}{RESET_COLOUR}")


def run_command(command, source_functions=True):
    # https://stackoverflow.com/questions/3777301/how-to-call-a-shell-script-from-python-code#3777308
    prefix = ""
    if source_functions:
        prefix = "source install_functions.sh; "

    command = f"{prefix}{command}"
    info(f"Running {command}")
    subprocess.check_call(["bash", "-c", f"cd {os.getcwd()}; {command}"])


def reinstall(need_to_reboot):
    def stage1():
        # First stage of the process
        run_command("install_ros")

    def stage2():
        # Second stage of the process

        info("Setting up computer vision package")
        # These changes are required to be able to build
        # Required even if not running CV

        # Copy over configs for "cam_params" and "camera_info"
        for fname in ["cam_params", "camera_info"]:
            if not os.path.isfile(f"{CV_CONFIG_PATH}/{fname}.yaml"):
                info(
                    "Using example {fname}.yaml. If you are using a different camera, please replace with your own config"
                )
                run_command(
                    f"cp {CV_CONFIG_PATH}/{fname}_template.yaml {CV_CONFIG_PATH}/{fname}.yaml",
                    source_functions=False,
                )

        run_command("install_workspace")

    if need_to_reboot:
        lockfile = ".need_to_resume_installation"
        if not os.path.isfile(lockfile):
            stage1()
            run_command(f"touch {lockfile}", source_functions=False)
            # remember to resume
            info("Please reboot and run the same command again")
        else:
            stage2()
            run_command(f"rm {lockfile}", source_functions=False)
    else:
        stage1()
        stage2()

    info("Done! Now you can install additional packages using this script")


def run_first_install(args):
    reinstall(need_to_reboot=True)


def run_reinstall(args):
    reinstall(need_to_reboot=False)


def setup_cv_training(args):

    # Write the roboflow key if needed
    if not os.path.isfile(CV_TRAIN_PATH + "/.roboflow_key"):
        key = input("Please enter your Roboflow key >")
        with open(CV_TRAIN_PATH + "/.roboflow_key", "w") as f:
            f.write(key)

    # Create a venv specifically for computer vision training
    if not os.path.isdir(f"{CV_TRAIN_PATH}/.venv"):
        run_command(
            f"cd {CV_TRAIN_PATH}; python3 -m venv .venv",
            source_functions=False,
        )
    run_command(
        f"cd {CV_TRAIN_PATH}; source .venv/bin/activate; pip install -r requirements.txt",
        source_functions=False,
    )

def setup_moveit(args):
    run_command("install_moveit")


parser = argparse.ArgumentParser(
    prog="Helper scripts", description="Installation helper for OxARM repo"
)
subparsers = parser.add_subparsers(help="Runs possible commands")

first_install_parser = subparsers.add_parser(
    "first-install", help="Run first setup of packages"
)
first_install_parser.set_defaults(func=run_first_install)

reinstall_parser = subparsers.add_parser(
    "reinstall", help="Reinstall the required packages"
)
reinstall_parser.set_defaults(func=run_reinstall)

setup_cv_parser = subparsers.add_parser(
    "setup_cv_training", help="Sets up CV to be able to train the model"
)
setup_cv_parser.set_defaults(func=setup_cv_training)

setup_moveit_parser = subparsers.add_parser(
    "setup_moveit", help="Sets up moveit packages"
)
setup_moveit_parser.set_defaults(func=setup_moveit)

args = parser.parse_args()
try:
    # Call the function
    args.func(args)
except AttributeError as e:
    print("No subcommand passed")
