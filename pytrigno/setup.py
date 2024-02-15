from setuptools import setup, find_packages

INSTALL_REQUIRES = [
    'numpy',
]

setup(
    name='pytrigno',
    version='0.1',
    description='Simple Python interface to the Delsys wireless EMG system.',

    classifiers=[
        'Development Status :: 4 - Beta',
        'License :: OSI Approved :: BSD License',
        'Intended Audience :: Science/Research',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 3'
    ],
    keywords='electromyography',

    packages=find_packages(),
    install_requires=INSTALL_REQUIRES,

    url='https://github.com/ucdrascal/pytrigno',
    author='Kenneth Lyons',
    author_email='ixjlyons@gmail.com',
    license='MIT',

)
