import os
import shutil
import os.path,time

file1='../lib/phy6200/phyos/keil/'
file2='../../../../components/ethermind/lib/meshlibs/phyos/keil/'

fileName ={
        'libethermind_ecdh.lib',
        'libethermind_mesh_core.lib',
        'libethermind_mesh_models.lib',
        'libethermind_utils.lib'
        }

for fi in fileName:
    print(file1+fi,time.ctime(os.path.getmtime(file1+fi)))
    print(file2+fi,time.ctime(os.path.getmtime(file2+fi)))
    shutil.copy2(file1+fi,file2+fi)
