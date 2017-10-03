from setuptools import setup, find_packages
from pip.req import parse_requirements


def main():
    setup(name='poorprof',
          version='0.1',
          description='Poor profiling library',
          py_modules=['poorprof'],
          )

if __name__ == '__main__':
    main()
