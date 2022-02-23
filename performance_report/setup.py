from setuptools import setup

package_name = 'performance_report'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={package_name: ['templates/*.html']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    # install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apex.AI',
    maintainer_email='tooling@apex.ai',
    url='https://github.com/ApexAI/performance_test',
    description='Apex.AI performance_test runner, plotter, and reporter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = performance_report.generate_experiment_commands:main',
            'runner = performance_report.run_experiment:main',
            'plotter = performance_report.generate_plots:main',
            'reporter = performance_report.generate_report:main',
        ],
    },
)
