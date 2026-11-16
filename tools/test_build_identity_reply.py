#!/usr/bin/env python3
"""Execute the actual BUILD formatter/dispatch with synthetic app metadata.

The installed ESP-IDF getter is also compiled unchanged with its nine-character
cache populated, demonstrating why a 65-byte caller buffer is insufficient.
No target build, serial connection, flash or hardware operation is performed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

from test_main_config_commit import function

ROOT = Path(__file__).resolve().parents[1]


def sha(data): return hashlib.sha256(data).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--idf-path', type=Path,
                        default=Path(os.environ.get('IDF_PATH', str(Path.home() / 'esp/esp-idf-v5.5.5'))))
    parser.add_argument('--manifest', type=Path)
    args = parser.parse_args(argv)
    if args.manifest and args.manifest.exists(): raise FileExistsError('existing test evidence')
    paths = [ROOT / 'main/main.c', args.idf_path / 'components/esp_app_format/esp_app_desc.c',
             args.idf_path / 'components/esp_app_format/include/esp_app_desc.h']
    sources = [path.read_text() for path in paths]
    source, sdk_source, sdk_header = sources
    formatter = function(source, 'static esp_err_t sensorarrayFormatBuildReply(')
    branches = []
    for name in ('sensorarrayEarlyRuntimeQueryCommand', 'sensorarrayRuntimeQueryCommand'):
        query = function(source, 'static esp_err_t ' + name + '(')
        branch = function(query, '    if (strcmp(command, "BUILD?") == 0)')
        assert 'return sensorarrayFormatBuildReply(response, responseSize);' in branch
        branches.append('static esp_err_t ' + name + '(const char *command, char *response, size_t responseSize) {\n' +
                        branch + '\nreturn ESP_ERR_NOT_SUPPORTED;\n}')
    descriptor = sdk_header[sdk_header.index('typedef struct {'):sdk_header.index('} esp_app_desc_t;') + len('} esp_app_desc_t;')]
    sdk_init = function(sdk_source, 'static void esp_app_format_init_elf_sha256(')
    sdk_getter = function(sdk_source, 'int esp_app_get_elf_sha256(')
    assert 'sizeof(app_elf_sha256_str)' in sdk_getter
    unit = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#define CONFIG_IDF_TARGET "esp32s3"
#define SENSORARRAY_PROTOCOL_VERSION 2u
#define CONFIG_APP_RETRIEVE_LEN_ELF_SHA 9
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/* DESCRIPTOR */
static esp_app_desc_t esp_app_desc;
static int descriptor_missing;
static const esp_app_desc_t *esp_app_get_description(void)
{ return descriptor_missing ? NULL : &esp_app_desc; }
static const char *esp_get_idf_version(void) { return "v5.5.5"; }
/* Synthetic already-initialized SDK panic cache; not a complete ELF hash. */
_Alignas(int) char app_elf_sha256_str[CONFIG_APP_RETRIEVE_LEN_ELF_SHA + 1] = "9a16d7673";
/* SDK_INIT */
/* SDK_GETTER */
/* FORMATTER */
/* DISPATCH */
int main(void)
{
    char old[65]; memset(old, '!', sizeof(old));
    assert(esp_app_get_elf_sha256(old, sizeof(old)) == 10);
    assert(strlen(old) == 9 && strcmp(old, "9a16d7673") == 0 && old[10] == '!');

    static const char version32[] = "0123456789ABCDEFGHIJKLMNOPQRSTUV";
    static const char hash64[] = "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";
    assert(sizeof(version32) == 33 && sizeof(hash64) == 65);
    memcpy(esp_app_desc.version, version32, 32); /* Deliberately no NUL. */
    memset(esp_app_desc.project_name, 'Z', sizeof(esp_app_desc.project_name));
    for (size_t i = 0; i < sizeof(esp_app_desc.app_elf_sha256); ++i) esp_app_desc.app_elf_sha256[i] = (uint8_t)i;
    char out[512], early[512], normal[512], expected[512];
    snprintf(expected, sizeof(expected),
        "BUILD,idf=v5.5.5,target=esp32s3,project=SensorArray,proto=2,version=%s,elfSha256=%s\n", version32, hash64);
    assert(sensorarrayFormatBuildReply(out, sizeof(out)) == ESP_OK);
    assert(strcmp(out, expected) == 0);
    const char *hash = strstr(out, ",elfSha256=") + strlen(",elfSha256=");
    assert(strcspn(hash, "\n") == 64 && hash[64] == '\n' && hash[65] == '\0');
    const char *version = strstr(out, ",version=") + strlen(",version=");
    assert(strcspn(version, ",") == 32);

    assert(sensorarrayEarlyRuntimeQueryCommand("BUILD?", early, sizeof(early)) == ESP_OK);
    assert(sensorarrayRuntimeQueryCommand("BUILD?", normal, sizeof(normal)) == ESP_OK);
    assert(strcmp(early, expected) == 0 && strcmp(normal, expected) == 0);
    assert(sensorarrayEarlyRuntimeQueryCommand("other?", early, sizeof(early)) == ESP_ERR_NOT_SUPPORTED);

    size_t length = strlen(expected);
    assert(sensorarrayFormatBuildReply(out, length + 1u) == ESP_OK);
    assert(strcmp(out, expected) == 0);
    memset(out, '!', sizeof(out));
    assert(sensorarrayFormatBuildReply(out, length) == ESP_ERR_INVALID_SIZE && out[0] == '\0');
    assert(out[length] == '!');
    char one[2] = {'X', 'Y'};
    assert(sensorarrayFormatBuildReply(one, 1) == ESP_ERR_INVALID_SIZE && one[0] == '\0' && one[1] == 'Y');
    assert(sensorarrayFormatBuildReply(NULL, 512) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayFormatBuildReply(out, 0) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayEarlyRuntimeQueryCommand("BUILD?", early, 1) == ESP_ERR_INVALID_SIZE && early[0] == '\0');
    assert(sensorarrayRuntimeQueryCommand("BUILD?", normal, 1) == ESP_ERR_INVALID_SIZE && normal[0] == '\0');

    memset(esp_app_desc.app_elf_sha256, 0xff, sizeof(esp_app_desc.app_elf_sha256));
    memcpy(app_elf_sha256_str, "000000000", sizeof(app_elf_sha256_str));
    assert(sensorarrayFormatBuildReply(out, sizeof(out)) == ESP_OK);
    hash = strstr(out, ",elfSha256=") + strlen(",elfSha256=");
    for (size_t i = 0; i < 64; ++i) assert(hash[i] == 'f');
    assert(hash[64] == '\n');
    descriptor_missing = 1; out[0] = 'X';
    assert(sensorarrayFormatBuildReply(out, sizeof(out)) == ESP_ERR_INVALID_STATE && out[0] == '\0');
    puts("BUILD identity formatter: PASS (SDK 65-byte buffer still 9 cached characters; descriptor 64 hex, version32, both dispatches, exact fit, short/null buffers)");
    return 0;
}
'''
    for marker, value in [('DESCRIPTOR', descriptor), ('SDK_INIT', sdk_init), ('SDK_GETTER', sdk_getter),
                          ('FORMATTER', formatter), ('DISPATCH', '\n\n'.join(branches))]:
        unit = unit.replace('/* ' + marker + ' */', value)
    with tempfile.TemporaryDirectory(prefix='sensorarray-build-identity-') as directory:
        tmp = Path(directory); c_file, executable = tmp / 'unit.c', tmp / 'test'
        c_file.write_text(unit)
        command = [os.environ.get('CC', '/usr/bin/gcc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                   '-O1', '-fno-strict-aliasing', '-I', str(ROOT / 'tests/host/include'), str(c_file), '-o', str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True, timeout=20)
    report = {'status': 'PASS', 'fixture_type': 'synthetic_descriptor_no_hardware',
              'formatter_source_sha256': sha(formatter.encode()),
              'sdk_getter_source_sha256': sha(sdk_getter.encode()),
              'input_sha256': {str(path): sha(text.encode()) for path, text in zip(paths, sources)},
              'test_source_sha256': sha(Path(__file__).read_bytes()),
              'generated_C_sha256': sha(unit.encode()), 'compiler_argv': command,
              'sdk_cache_characters': 9, 'caller_buffer_bytes': 65, 'BUILD_ELF_hex_characters': 64,
              'dispatches_executed': ['early', 'normal'], 'target_build_run': False, 'hardware_accessed': False}
    if args.manifest:
        with args.manifest.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps(report))


if __name__ == '__main__': main()
