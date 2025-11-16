from setuptools import find_packages, setup
import os

package_name = 'rbx1_description'

data_files = [
    (f"share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]

# install whole directories (meshes, urdf, config, launch, etc.)
def _collect_data_files(package_name, src_dir):
    result = {}
    if not os.path.isdir(src_dir):
        return result
    for root, _, files in os.walk(src_dir):
        if not files:
            continue
        rel_root = os.path.relpath(root, ".")  # e.g. "meshes" or "meshes/collision"
        dest = f"share/{package_name}/{rel_root}"
        paths = [os.path.join(root, f) for f in files if os.path.isfile(os.path.join(root, f))]
        if paths:
            result.setdefault(dest, []).extend(paths)
    return result

# gather and append data files
collected = {}
for d in ("meshes", "urdf", "launch", "config"):
    collected.update(_collect_data_files(package_name, d))

data_files += list(collected.items())

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='vscode@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
