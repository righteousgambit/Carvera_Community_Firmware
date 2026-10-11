#pragma once
#define MBED_FATFILESYSTEM_H
#define MBED_FATFILEHANDLE_H
#define MBED_FATDIRHANDLE_H
#define FFSDEBUG_ENABLED 0
#define FFSDEBUG(...)
#define FATFS_PATH_MAX 260
#include <stdint.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include "ff.h"
namespace mbed {
class FileHandle { public: virtual ~FileHandle() {} };
class DirHandle { public: virtual ~DirHandle() {} };
class FileSystemLike {
public:
    const char* _name;
    explicit FileSystemLike(const char* name): _name(name) {}
};
class FATFileHandle: public FileHandle {
public:
    FIL_t* file;
    explicit FATFileHandle(FIL_t* value): file(value) {}
};
class FATDirHandle: public DirHandle { public: explicit FATDirHandle(DIR_t) {} };
class FATFileSystem: public FileSystemLike {
public:
    explicit FATFileSystem(const char*);
    ~FATFileSystem();
    FileHandle* open(const char*, int);
    int remove(const char*);
    int rename(const char*, const char*);
    int format();
    DirHandle* opendir(const char*);
    int mkdir(const char*, mode_t);
    FATFS _fs;
    static FATFileSystem* _ffs[_DRIVES];
    int _fsid;
};
}
inline void error(const char*, ...) { throw "unexpected filesystem mount failure"; }
