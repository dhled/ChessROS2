from setuptools import setup

package_name = "orchesstrateur"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="srobert",
    maintainer_email="simon.robert@irt-jules-verne.fr",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchesstrateur = orchesstrateur.main_node:main",
        ],
    },
)
