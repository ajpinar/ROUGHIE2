from distutils.core import setup, Extension

module1 = Extension('pymu',
	sources=['pymu.c','vn200.c','vndevice.c','arch/linux/vncp_services.c'],
	include_dirs=['../include'])

setup (name = 'PyMU',
        version = '1.0',
        description = 'Tony\'s package',
        ext_modules = [module1])