from setuptools import setup
from setuptools import find_packages
from setuptools.command.build_ext import build_ext
from distutils.core import Extension
import os

ext_modules = [
    Extension(
        name="ik_solver.ik_solver_py",  
        sources=[],                      
    )
]

# Definisci il setup
setup(
    name="ik_solver",
    version="0.1",
    author="Cesare Tonola",
    author_email="tonolacesare@gmail.com",
    description="IK solver using Pinocchio and pybind11",
    packages=find_packages(where="scripts"),
    package_dir={"": "scripts"},
    package_data={"ik_solver": ["ik_solver_py.so"]},
    include_package_data=True,
    ext_modules=ext_modules,
    zip_safe=False,
)

