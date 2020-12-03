from setuptools import setup

with open('README.md', 'r') as file:
  long_description = file.read()

install_requires = []
with open('requirements.txt') as file:
  install_requires = [line for line in file.read().splitlines() if len(line) > 0]

setup(
  name='ebsd',
  version='0.1.0',
  author='BlueQuartz Software, LLC',
  author_email='info@bluequartz.net',
  description='ebsd is a Python package for reading .ang and .ctf files',
  long_description=long_description,
  long_description_content_type='text/markdown',
  url='https://github.com/BlueQuartzSoftware/EbsdLib',
  py_modules=['ebsd'],
  license='BSD',
  platforms='any',
  classifiers=[
      'Programming Language :: Python :: 3',
      'License :: OSI Approved :: BSD License'
  ],
  python_requires='>=3.8',
  install_requires=install_requires
)
