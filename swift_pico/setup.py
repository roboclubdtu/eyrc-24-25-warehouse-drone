from setuptools import setup

package_name = 'swift_picp'

setup(
    name=package_name,
    version='0.0.1',
    packages=['scripts'],
    package_dir={'': '.'},
    install_requires=[],
    zip_safe=True,
    maintainer='Jonathan M.',
    maintainer_email='ionymikler@example.com',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any executable scripts here if needed
        ],
    },
)
