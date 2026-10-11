#include "stubs.h"
#include "SensorEvidenceStorage.h"
#include <errno.h>
#include <map>
#include <string>
#include <iostream>

static std::string directory;
static std::map<FIL_t*, FILE*> handles;
static FRESULT injected = FR_OK;
static unsigned creates = 0;

extern "C" FRESULT f_open(FIL_t* file, const TCHAR* name, BYTE mode) {
    if (injected != FR_OK) { errno = 0; return injected; }
    const std::string path = directory + "/" + (std::string(name).substr(3));
    const char* flags = (mode & FA_CREATE_ALWAYS) ? "w+b" : "r+b";
    FILE* host = ::fopen(path.c_str(), flags);
    // FatFs communicates its error through FRESULT, not errno.
    errno = 0;
    if (!host) return FR_NO_FILE;
    if (mode & FA_CREATE_ALWAYS) ++creates;
    handles[file] = host;
    return FR_OK;
}
extern "C" FRESULT f_mount(BYTE, FATFS*) { return FR_OK; }
extern "C" FRESULT f_lseek(FIL_t*, DWORD) { return FR_OK; }
extern "C" FRESULT f_unlink(const TCHAR*) { return FR_OK; }
extern "C" FRESULT f_rename(const TCHAR*, const TCHAR*) { return FR_OK; }
extern "C" FRESULT f_mkfs(BYTE, BYTE, UINT) { return FR_OK; }
extern "C" FRESULT f_opendir(DIR_t*, const TCHAR*) { return FR_OK; }
extern "C" FRESULT f_mkdir(const TCHAR*) { return FR_OK; }

static mbed::FATFileSystem filesystem("sd");
struct AdapterIO: sensor_evidence::FileIO {
    static FILE* open(const char* path, const char* mode) {
        const int flags = mode[0] == 'w' ? O_RDWR | O_CREAT | O_TRUNC : O_RDWR;
        auto* handle = static_cast<mbed::FATFileHandle*>(filesystem.open(path, flags));
        if (!handle) return nullptr;
        FILE* file = handles.at(handle->file);
        handles.erase(handle->file);
        delete handle->file;
        delete handle;
        return file;
    }
};

static bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << "\n";
    return condition;
}
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    directory = argv[1];
    uint32_t counter = 0;
    if (!check(sensor_evidence::advance_boot_counter<AdapterIO>("sensor-boot.dat", counter) && counter == 1,
               "first boot must create its missing counter through the FAT adapter")) return 1;
    if (!check(sensor_evidence::advance_boot_counter<AdapterIO>("sensor-boot.dat", counter) && counter == 2,
               "second boot must preserve and advance the counter")) return 1;
    const unsigned before = creates;
    for (FRESULT failure: {FR_DISK_ERR, FR_NOT_READY, FR_DENIED, FR_WRITE_PROTECTED}) {
        injected = failure;
        if (!check(!sensor_evidence::advance_boot_counter<AdapterIO>("sensor-boot.dat", counter) && counter == 0 && creates == before,
                   "hard open failures must not create or truncate an existing counter")) return 1;
    }
    injected = FR_OK;
    if (!check(sensor_evidence::advance_boot_counter<AdapterIO>("sensor-boot.dat", counter) && counter == 3,
               "hard failures must leave the reserved boot identity intact")) return 1;
    injected = FR_NO_PATH;
    errno = 0;
    if (!check(filesystem.open("missing/counter", O_RDWR) == nullptr && errno == ENOENT,
               "missing paths must report ENOENT")) return 1;
    injected = FR_OK;
    errno = 0;
    if (!check(filesystem.open(std::string(300, 'x').c_str(), O_RDWR) == nullptr && errno == ENAMETOOLONG,
               "long paths must not masquerade as absent files")) return 1;
    std::cout << "FAT open and first-boot storage regression passed\n";
}
