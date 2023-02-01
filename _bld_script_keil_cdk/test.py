import os


def check_dirs():
	curpath = os.path.dirname(os.path.realpath("..\_bld_script_yml\sdk_build.yml"))
	cdkyml = os.path.exists('..\_bld_script_yml\cdk-make.yml')
	keilyml = os.path.exists('..\_bld_script_yml\sdk_bld_tool.yml')
	
	flg = 0;
	
	if cdkyml:
		flg = 1
	elif keilyml:
		flg = 2
	else:
		pass		

	print(curpath)
	print(cdkyml)

	return flg

if __name__ == "__main__":
    ret = check_dirs()
    print(ret)
