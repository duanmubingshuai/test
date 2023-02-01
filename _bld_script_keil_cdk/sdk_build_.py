#! /usr/bin/env python3
import os, sys, getopt
#from common.legalexception import *
from sdk_build_cdk import *
from sdk_build_keil import *

def help(prj = None):
	print('sdk_build.py: Build PhyPlus BLE SDK')
	print('useage:')
	print('1:')
	print('	sdk_build.py [-help [projectname]] [-clear] [-ver 1.1.1.b] [-path sdk_path][-list] [[-l ymlfile] [-b [projectname]|[all]]]') 
	
def main(argv):

	try:
		opts, args = getopt.getopt(sys.argv[1:], 'h:k:c:', ["help", "kel", "clear", "cdk=", "sheet=", "all="])
		# print("sys.argv[1:] is %s"%(sys.argv[1:]))
		# print("opts is %s"%(opts))
	except getopt.GetoptError:
		# print help information and exit:
		# usage()
		help()
		#sys.exit(2)
		return

	for opt, arg in opts:
		# print('opt and arg in opts are ')
		# print(opt, arg)
		if opt in ('-', '-h', '--help', '-help'):
			help()
			return

		# if opt in ('-c', '-k','-cdk', '-kel'):
		if opt in ('-clear'):
			os.system('python .\sdk_build_cdk.py -clear')
			break

		# if opt in ('-cdk'):
		if opt == '--cdk':
			# os.system('python .\sdk_build_cdk.py -clear')
			os.system('python .\sdk_build_cdk.py -l ' +arg +' -'+args[0]+ 'all')
			break

		if opt == '--sheet':
			# os.system('python .\_reg_gen.py --verbose --short --sheet=' +args[0] +' --access=access --prefix=' +args[1] +' --xlsname=' +args[2] +' outfile')   # with modified _reg_gen.py
			os.system('python .\_reg_gen.py --verbose --short --sheet=' +arg +' --access=_JACK_REG --prefix=' +args[1] +' --xlsname=' +args[0] +' outfile')   # with modified _reg_gen.py
			break

		elif opt == '--all':
			sheetname = get_sheet(arg)
			lens = len(sheetname)
			xlsname = arg
			# print(set)
			# print(lens)
			for i in range(lens):
				os.system('python .\_reg_gen.py --verbose --short --sheet=' +sheetname[i] +' --access=_JACK_REG --prefix=' +args[0] +' --xlsname=' +xlsname +' outfile') 
			# print('"set nums is %d\n" %(len(set))')
			break
	
if __name__ == '__main__':
	#sys.argv = ['.\\sdk_build.py', '-lcfg', 'newname','-list']   #ok  config file not exist
	#sys.argv = ['.\\sdk_build.py', '-lcfg', '-list']  #no config file name  #ok
	#sys.argv = ['.\\sdk_build.py', '-lcfg', 'sdk_build', '-list']  # ok #config file exist
	# clear_commit_id()
	# commit_date()
	# commit_id()
	main(sys.argv)
