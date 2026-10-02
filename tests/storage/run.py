from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[2]
s=(r/'OpenPuck/storage.cpp').read_text().replace('0xED000','(uintptr_t)testFlash')
test='''
#include <cassert>
#include <algorithm>
bool mountOk=false,failWrite=false,failRead=false,failRename=false;
int formats=0;
std::map<std::string,std::vector<uint8_t>> files;
InternalFileSystem InternalFS;
uint32_t testFlash[7168];
int main(){
 std::fill(testFlash,testFlash+7168,0xFFFFFFFF);testFlash[100]=0;
 assert(!storageBegin());assert(formats==0);assert(g_storageState==0);
 std::fill(testFlash,testFlash+7168,0xFFFFFFFF);
 assert(storageBegin());assert(formats==1);assert(g_storageState==2);
 assert(storageBegin());assert(formats==1);
 const uint8_t old[]={1,2,3};const uint8_t next[]={8,9,10,11};
 files["/cfg.bin"]={old,old+3};
 failWrite=true;assert(!storageWriteFile("/cfg.bin","/cfg.tmp",next,4));
 assert(files["/cfg.bin"]==std::vector<uint8_t>(old,old+3));assert(g_storageState==3);
 failWrite=false;failRead=true;assert(!storageWriteFile("/cfg.bin","/cfg.tmp",next,4));
 assert(files["/cfg.bin"]==std::vector<uint8_t>(old,old+3));
 failRead=false;failRename=true;assert(!storageWriteFile("/cfg.bin","/cfg.tmp",next,4));
 assert(files["/cfg.bin"]==std::vector<uint8_t>(old,old+3));
 failRename=false;assert(storageWriteFile("/cfg.bin","/cfg.tmp",next,4));
 assert(files["/cfg.bin"]==std::vector<uint8_t>(next,next+4));assert(g_storageState==1);
 files["/bonds.bin"]={old,old+3};failWrite=true;
 assert(!storageWriteFile("/bonds.bin","/bonds.tmp",next,4));
 assert(files["/bonds.bin"]==std::vector<uint8_t>(old,old+3));
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(s+test)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(r/'tests/storage/stubs'),'-I'+str(r/'OpenPuck'),str(p),'-o',td+'/test'],check=True)
 subprocess.run([td+'/test'],check=True,env={'ASAN_OPTIONS':'detect_leaks=0'})
print('Storage mount and interrupted-save tests passed')
