#!/usr/bin/env python3
"""Check the real project/SDK task-wrapper Kconfig extension without building.

Uses the installed ESP-IDF Kconfig parser, actual SDK symbol, project extension,
and main.c compile guard. All generated configuration/header files live in /tmp.
Missing SDK/parser/configuration inputs produce explicit SKIP and exit 77.
No SDK Kconfig contents, credentials, or full sdkconfig are written to reports.
"""
from pathlib import Path
import argparse
import contextlib
import hashlib
import importlib.metadata
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap


SYMBOL = "FREERTOS_TASK_FUNCTION_WRAPPER"
GATES = ("IDF_TARGET_ESP32S3", "COMPILER_OPTIMIZATION_PERF",
         "COMPILER_OPTIMIZATION_DEBUG", "ESP_COREDUMP_ENABLE",
         "ESP_SYSTEM_PANIC_GDBSTUB", "ESP_SYSTEM_GDBSTUB_RUNTIME")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def skip(reason):
    print(json.dumps({"status": "SKIP", "reason": reason,
                      "target_build": False, "hardware": False}))
    return 77


def symbol_block(source, name):
    lines = source.splitlines(keepends=True)
    positions = [i for i, line in enumerate(lines)
                 if re.fullmatch(r"\s*config " + re.escape(name) + r"\s*", line)]
    assert len(positions) == 1, (name, positions)
    start = positions[0]
    indent = len(lines[start]) - len(lines[start].lstrip())
    end = len(lines)
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if line.strip() and len(line) - len(line.lstrip()) <= indent and re.match(
                r"\s*(config|menuconfig|menu|endmenu|choice|endchoice|if|endif)\b", line):
            end = i
            break
    return "".join(lines[start:end])


def parser_or_reexec(already_reexecuted):
    try:
        import kconfiglib
        return kconfiglib
    except ImportError:
        if already_reexecuted:
            return None
        candidates = []
        if os.environ.get("IDF_PYTHON_ENV_PATH"):
            candidates.append(Path(os.environ["IDF_PYTHON_ENV_PATH"]) / "bin/python")
        candidates.extend(sorted((Path.home() / ".espressif/python_env").glob(
            "idf5.5*_env/bin/python")))
        for python in candidates:
            if python.is_file() and subprocess.run(
                    [str(python), "-c", "import kconfiglib"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                os.execv(str(python), [str(python), __file__, *sys.argv[1:],
                                      "--using-idf-python"])
        return None


def check_guard(root, compiler):
    source = (root / "main/main.c").read_text()
    matches = re.findall(r"^#if[^\n]*FREERTOS_TASK_FUNCTION_WRAPPER[^\n]*\n"
                         r"#error[^\n]*\n#endif\n", source, re.M)
    assert len(matches) == 1, "Expected one real main.c task-wrapper guard"
    guard = matches[0]
    assert not re.search(r"^\s*#\s*define\s+CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER\b",
                         source, re.M), "An application define cannot configure SDK port.c"
    results = []
    for s3, perf, wrapper in ((1, 1, 1), (1, 1, 0), (1, 1, None),
                              (0, 1, 0), (1, 0, 0), (0, 0, None)):
        prefix = f"#define CONFIG_IDF_TARGET_ESP32S3 {s3}\n#define CONFIG_COMPILER_OPTIMIZATION_PERF {perf}\n"
        if wrapper is not None:
            prefix += f"#define CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER {wrapper}\n"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                   "-fsyntax-only", "-x", "c", "-"]
        run = subprocess.run(command, input=prefix + guard + "int guard_check(void) { return 0; }\n",
                             text=True, capture_output=True)
        rejected = bool(s3 and perf and not wrapper)
        assert (run.returncode != 0) == rejected, (s3, perf, wrapper, run.stderr)
        if rejected:
            assert "SensorArray requires the FreeRTOS task function wrapper" in run.stderr
        results.append({"s3": s3, "perf": perf, "wrapper": wrapper,
                        "expected_rejection": rejected, "exit_code": run.returncode})
    return results


def run_checks(args, root, idf, kconfiglib):
    from kconfiglib.core import expr_str
    compat = root / "main/kconfigs/Kconfig.compat"
    project_block = symbol_block(compat.read_text(), SYMBOL)
    sdk_kconfig = idf / "components/freertos/Kconfig"
    sdk_block = textwrap.dedent(symbol_block(sdk_kconfig.read_text(), SYMBOL))
    fixture = root / "tests/fixtures/performance_task_wrapper_cases.json"
    cases = json.loads(fixture.read_text())["cases"]
    config_env = json.loads(args.config_env.read_text())
    assert Path(config_env["IDF_PATH"]).resolve() == idf.resolve(), "SDK/config.env mismatch"
    config_env["IDF_PATH"] = str(idf)
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        return {"status": "SKIP", "reason": "native C compiler unavailable"}
    inputs = [compat, root / "main/main.c", root / "main/Kconfig.projbuild",
              sdk_kconfig, idf / "Kconfig", args.sdkconfig, args.config_env,
              fixture, Path(__file__), idf / "components/freertos/linker.lf",
              idf / "components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c"]
    before = {str(p): digest(p) for p in inputs}
    result = {"status": "PASS", "kind": "production_Kconfig_and_guard_regression",
              "target_build": False, "hardware": False,
              "parser_version": importlib.metadata.version("esp-idf-kconfig"),
              "input_sha256": before, "matrix": [], "full_tree": []}
    # The installed parser's relative source handling uses srctree. CWD is also
    # aligned so its duplicate-definition diagnostics name the actual SDK file.
    os.environ.update({k: str(v) for k, v in config_env.items()})
    os.environ["srctree"] = str(idf)
    os.chdir(idf)
    with tempfile.TemporaryDirectory(prefix="sensorarray-task-wrapper-") as tmp:
        work = Path(tmp)
        minimal = 'mainmenu "Synthetic wrapper boundary"\n' + "".join(
            f'config {name}\n    bool "{name}"\n' for name in GATES) + sdk_block
        for case in cases:
            for extended in (False, True):
                path = work / "matrix.Kconfig"
                path.write_text(minimal + ("\n" + project_block if extended else ""))
                k = kconfiglib.Kconfig(str(path), warn=False, info=False)
                for field, name in zip(("s3", "perf", "debug"), GATES[:3]):
                    k.syms[name].set_value(2 if case[field] else 0)
                if case["extra"]:
                    k.syms[case["extra"]].set_value(2)
                expected = bool(case["debug"] or case["extra"] or
                                (extended and case["s3"] and case["perf"]))
                got = k.syms[SYMBOL].str_value
                assert got == ("y" if expected else "n"), (case, extended, got)
                result["matrix"].append({**case, "extension": extended, "wrapper": got})
        all_values = {}
        components = work / "components.in"
        components.write_text("".join(f'source "{p}"\n' for p in
                              config_env["COMPONENT_KCONFIGS"].split(";") if p))
        os.environ["COMPONENT_KCONFIGS_SOURCE_FILE"] = str(components)
        for kind in ("baseline", "extension"):
            # Mirror only repository Kconfig inputs. SDK source files are read
            # in place; no SDK tree, config.env, sdkconfig, or build is edited.
            main_copy = work / kind / "main"
            main_copy.mkdir(parents=True)
            shutil.copy2(root / "main/Kconfig.projbuild", main_copy / "Kconfig.projbuild")
            shutil.copytree(root / "main/kconfigs", main_copy / "kconfigs")
            if kind == "baseline":
                q = main_copy / "kconfigs/Kconfig.compat"
                q.write_text(q.read_text().replace(project_block, "", 1))
            projbuild = work / (kind + ".projbuild.in")
            paths = [str(main_copy / "Kconfig.projbuild") if Path(p).resolve() ==
                     (root / "main/Kconfig.projbuild").resolve() else p
                     for p in config_env["COMPONENT_KCONFIGS_PROJBUILD"].split(";") if p]
            projbuild.write_text("".join(f'source "{p}"\n' for p in paths))
            os.environ["COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE"] = str(projbuild)
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured), contextlib.redirect_stderr(captured):
                k = kconfiglib.Kconfig(str(idf / "Kconfig"), warn=False, info=True)
                k.load_config(str(args.sdkconfig))
                k.syms["COMPILER_OPTIMIZATION_PERF"].set_value(2)
                assert k.syms["IDF_TARGET_ESP32S3"].str_value == "y", "Full-tree check requires S3 config.env"
                assert all(k.syms[n].str_value == "n" for n in GATES[3:]), "Full-tree control must not enable core dump or GDB"
                all_values[kind] = {s.name: s.str_value for s in k.unique_defined_syms}
                header = work / (kind + ".h")
                k.write_autoconf(str(header))
            wrapper = k.syms[SYMBOL]
            defined = "#define CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER 1" in header.read_text()
            assert defined == (kind == "extension")
            result["full_tree"].append({"kind": kind, "wrapper": wrapper.str_value,
                "direct_dependency": expr_str(wrapper.direct_dep), "visibility": wrapper.visibility,
                "definition_count": len(wrapper.nodes), "generated_header_wrapper_y": defined,
                "generated_header_sha256": digest(header), "parser_info": captured.getvalue()})
        changes = {name: [all_values["baseline"].get(name), value]
                   for name, value in all_values["extension"].items()
                   if all_values["baseline"].get(name) != value}
        assert changes == {SYMBOL: ["n", "y"]}, changes
        result["full_tree_symbol_changes"] = changes
        result["effective_symbols_compared"] = len(all_values["extension"])
    result["guard_compile_cases"] = check_guard(root, compiler)
    assert before == {str(p): digest(p) for p in inputs}, "Inputs changed during test; rerun after freeze"
    result["inputs_unchanged"] = True
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--idf-path", type=Path, default=Path(os.environ.get(
        "IDF_PATH", "/home/csc/esp/esp-idf-v5.5.5")))
    parser.add_argument("--config-env", type=Path, default=root / "build/config.env")
    parser.add_argument("--sdkconfig", type=Path, default=root / "sdkconfig")
    parser.add_argument("--report", type=Path, help="Exclusive-create compact JSON evidence")
    parser.add_argument("--using-idf-python", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.idf_path, args.config_env, args.sdkconfig = (p.resolve() for p in
                                                   (args.idf_path, args.config_env, args.sdkconfig))
    if args.report:
        args.report = args.report.resolve()
        if args.report.exists():
            parser.error("report already exists; refusing to overwrite evidence")
    for p in (args.idf_path / "Kconfig", args.config_env, args.sdkconfig):
        if not p.is_file():
            return skip(f"Required SDK/configuration input unavailable: {p}")
    kconfiglib = parser_or_reexec(args.using_idf_python)
    if kconfiglib is None:
        return skip("Installed ESP-IDF Kconfig parser unavailable; no installation attempted")
    result = run_checks(args, root, args.idf_path, kconfiglib)
    if args.report:
        with args.report.open("x") as stream:
            json.dump(result, stream, indent=2)
            stream.write("\n")
    print(json.dumps({"status": result["status"], "matrix_cases": len(result.get("matrix", [])),
                      "guard_cases": len(result.get("guard_compile_cases", [])),
                      "full_tree_symbol_changes": result.get("full_tree_symbol_changes"),
                      "target_build": False, "hardware": False, "reason": result.get("reason")}))
    return 0 if result["status"] == "PASS" else 77


if __name__ == "__main__":
    raise SystemExit(main())
