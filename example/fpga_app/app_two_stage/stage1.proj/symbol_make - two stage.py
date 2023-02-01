
import os
import sys
import time

def ref_from_unref(ref_name):
	fp_unref = open(ref_name)
	fp_ref = open('ck802_otpsym_ref.txt', 'a+')
	#load ref
	fp_ref.seek(0, 0)
	ref_tbl = []
	while(True):
		line = fp_ref.readline()
		print('~~~~~~~~~~~~~', line)
		refsym_str = line
		if(len(line) <= 0):
			break
		if(len(line) <= 3):
			continue
		if(line[0] == '#'):
			continue
		if(line[:2] == '0x'):
			refsym_str = line.split()[2]
			print('===================',refsym_str)
		refsym_str = refsym_str.split()[0]
		ref_tbl.append(refsym_str)
	
	#seekend
	fp_ref.seek(0, 2)
	
	cmp_str = 'undefined reference to `'
	while(True):
		line = fp_unref.readline()
		if(len(line) <= 0):
			fp_unref.close()
			return
		if(cmp_str in line):
			#get symbol
			line = line[line.find(cmp_str) + len(cmp_str):]
			sym_str = line.split('\'')[0]
			#append sym_str
			if(sym_str not in ref_tbl):
				ref_tbl.append(sym_str)
				fp_ref.write(sym_str + '\n')
	fp_ref.close()
	fp_unref.close()
			
def load_sym_ref():
	fp_ref = open('ck802_otpsym_ref.txt')
	#load ref
	ref_tbl = []
	while(True):
		line = fp_ref.readline()
		refsym_str = line
		if(len(line) <= 0):
			break
		if(len(line) <= 3):
			continue
		if(line[0] == '#'):
			continue
		if(line[:2] == '0x'):
			refsym_str = line.split()[2]
		refsym_str = refsym_str.split()[0]
		ref_tbl.append(refsym_str)
	fp_ref.close()
	if(len(ref_tbl) ==0):
		print('ck802_otpsym_ref.txt did not found!')
	
	return ref_tbl

def add_to_sym_hfile(fp_sym_h, symname):
	print(symname,'|add_to_sym_hfile')
	if(fp_sym_h is None):
		fp_sym_h = open('Obj\\otp_sym_def.h', 'w')
		fp_sym_h.write('\n\n#ifndef __ROM_SYM_H__\n#define __ROM_SYM_H__\n#ifdef USE_ROMSYM_ALIAS\n\n\n')

	if(symname.__contains__("symrom")):
		print(symname, 'rom function not included')
	else:
		#fp_sym_h.write('#define '+ symname + ' _symrom_' + symname + '\n')
		fp_sym_h.write('#define ' + symname + ' _symotp_' + symname + '\n')
	return fp_sym_h
		
def close_sym_hfile(fp_sym_h):
	if(fp_sym_h is not None):
		fp_sym_h.write('\n\n#endif\n#endif\n\n')
		fp_sym_h.close()

def main(argv):
	if(argv[1] == '-ref'):
		ref_name = 'rom_sym_unref.log'
		if(len(argv) == 3):
			ref_name = argv[2]
		ref_from_unref(ref_name)
		return
		
	elfname = argv[1]
	os.system('del '+'_sym.txt 2>1>nul')
	cmd = 'csky-elfabiv2-nm.exe -g ' + elfname +' >_sym.txt'
	os.system(cmd)
	if(os.path.exists('_sym.txt') != True):
		print('\n\nfile is not exist:',self.m_path)
		return
	
	sym_ref_tbl = load_sym_ref()
	print(sym_ref_tbl)
	#convert to gdb symbol table
	fp = open('_sym.txt','r')
	fp_sym = open(elfname + '.ck802sym', 'w')
	fp_sym_h = None
	while(True):
		line = fp.readline()
		if(len(line) < 3):
			break
		itm = line.split()
		if(len(itm) != 3):
			continue
		symname = itm[2]
		if(symname == 'g_rfPhyTxPower'):
			print(symname)
			print(symname in sym_ref_tbl)
			print(sym_ref_tbl)
		if(symname in sym_ref_tbl):
			fp_sym_h = add_to_sym_hfile(fp_sym_h, symname)
		#symline = '_symrom_'+ itm[2] + ' = 0x' + itm[0] + ';\n'
		symline = '_symotp_' + itm[2] + ' = 0x' + itm[0] + ';\n'
		if (symline.__contains__("symrom")):
			print(symline, 'rom function not included')
		else:
			fp_sym.write(symline)
	fp.close()
	fp_sym.close()
	close_sym_hfile(fp_sym_h)
	os.system('del '+'_sym.txt 2>1>nul')
	os.system('copy '+'/Y ' +elfname + '.ck802sym'+  ' ..\\..\\..\\misc\\ 2>1>nul')
	os.system('copy '+'/Y ' +'Obj\\otp_sym_def.h' +  ' ..\\..\\..\\misc\\ 2>1>nul')
	

	


if __name__ == '__main__':
	main(sys.argv)
