from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['demo_traj'], # import 할 폴더 이름
    # package_dir={'': 'src'}             # 그 폴더가 src 안에 있음을 명시
)

setup(**d)