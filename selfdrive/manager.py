#!/usr/bin/env python3
import datetime
import importlib
import os
import sys
import fcntl
import errno
import signal
import shutil
import subprocess
import textwrap
import time
import traceback

from multiprocessing import Process
from typing import Dict, List

from common.basedir import BASEDIR
from common.spinner import Spinner
from common.text_window import TextWindow
from selfdrive.hardware import HARDWARE, EON, PC
from selfdrive.swaglog import cloudlog, add_logentries_handler

os.environ['BASEDIR'] = BASEDIR
sys.path.append(os.path.join(BASEDIR, "pyextra"))

from common.op_params import opParams
from common.travis_checker import travis
op_params = opParams()

traffic_lights = op_params.get('traffic_lights')


TOTAL_SCONS_NODES = 1040
WEBCAM = os.getenv("WEBCAM") is not None
PREBUILT = os.path.exists(os.path.join(BASEDIR, 'prebuilt'))


def unblock_stdout():
  # get a non-blocking stdout
  child_pid, child_pty = os.forkpty()
  if child_pid != 0:  # parent

    # child is in its own process group, manually pass kill signals
    signal.signal(signal.SIGINT, lambda signum, frame: os.kill(child_pid, signal.SIGINT))
    signal.signal(signal.SIGTERM, lambda signum, frame: os.kill(child_pid, signal.SIGTERM))

    fcntl.fcntl(sys.stdout, fcntl.F_SETFL, fcntl.fcntl(sys.stdout, fcntl.F_GETFL) | os.O_NONBLOCK)

    while True:
      try:
        dat = os.read(child_pty, 4096)
      except OSError as e:
        if e.errno == errno.EIO:
          break
        continue

      if not dat:
        break

      try:
        sys.stdout.write(dat.decode('utf8'))
      except (OSError, IOError, UnicodeDecodeError):
        pass

    # os.wait() returns a tuple with the pid and a 16 bit value
    # whose low byte is the signal number and whose high byte is the exit satus
    exit_status = os.wait()[1] >> 8
    os._exit(exit_status)


if __name__ == "__main__":
  unblock_stdout()

from common.spinner import Spinner
from common.text_window import TextWindow

if not (os.system("python3 -m pip list | grep 'scipy' ") == 0):
  os.system("cd /data/openpilot/installer/scipy_installer/ && ./scipy_installer")



# Run scons
spinner = Spinner(noop=(__name__ != "__main__"))
spinner.update("0")

def build():
  for retry in [True, False]:
    # run scons
    env = os.environ.copy()
    env['SCONS_PROGRESS'] = "1"
    env['SCONS_CACHE'] = "1"

    nproc = os.cpu_count()
    j_flag = "" if nproc is None else f"-j{nproc - 1}"
    scons = subprocess.Popen(["scons", j_flag], cwd=BASEDIR, env=env, stderr=subprocess.PIPE)

    compile_output = []

    # Read progress from stderr and update spinner
    while scons.poll() is None:
      try:
        line = scons.stderr.readline()  # type: ignore
        if line is None:
          continue
        line = line.rstrip()

        prefix = b'progress: '
        if line.startswith(prefix):
          i = int(line[len(prefix):])
          spinner.update("%d" % (70.0 * (i / TOTAL_SCONS_NODES)))
        elif len(line):
          compile_output.append(line)
          print(line.decode('utf8', 'replace'))
      except Exception:
        pass

    if scons.returncode != 0:
      # Read remaining output
      r = scons.stderr.read().split(b'\n')   # type: ignore
      compile_output += r

      if retry:
        if not os.getenv("CI"):
          print("scons build failed, cleaning in")
          for i in range(3, -1, -1):
            print("....%d" % i)
            time.sleep(1)
          subprocess.check_call(["scons", "-c"], cwd=BASEDIR, env=env)
          shutil.rmtree("/tmp/scons_cache", ignore_errors=True)
          shutil.rmtree("/data/scons_cache", ignore_errors=True)
        else:
          print("scons build failed after retry")
          process = subprocess.check_output(['git', 'pull'])
          os.system('reboot')
          sys.exit(1)
      else:
        # Build failed log errors
        errors = [line.decode('utf8', 'replace') for line in compile_output
                  if any([err in line for err in [b'error: ', b'not found, needed by target']])]
        error_s = "\n".join(errors)
        add_logentries_handler(cloudlog)
        cloudlog.error("scons build failed\n" + error_s)

        # Show TextWindow
        no_ui = __name__ != "__main__"
        error_s = "\n \n".join(["\n".join(textwrap.wrap(e, 65)) for e in errors])
        with TextWindow("openpilot failed to build\n \n" + error_s, noop=no_ui) as t:
          t.wait_for_exit()
        process = subprocess.check_output(['git', 'pull'])
        os.system('reboot')
        exit(1)
    else:
      break

if __name__ == "__main__" and not PREBUILT:
  build()

import cereal
import cereal.messaging as messaging

from common.params import Params
import selfdrive.crash as crash
from selfdrive.registration import register
from selfdrive.version import version, dirty
from selfdrive.loggerd.config import ROOT
from selfdrive.launcher import launcher
from selfdrive.hardware.eon.apk import update_apks, pm_apply_packages, start_offroad


# comment out anything you don't want to run
managed_processes = {
  "thermald": "selfdrive.thermald.thermald",
  "trafficd": ("selfdrive/trafficd", ["./trafficd"]),
  "traffic_manager": "selfdrive.trafficd.traffic_manager",
  "uploader": "selfdrive.loggerd.uploader",
  "deleter": "selfdrive.loggerd.deleter",
  "controlsd": "selfdrive.controls.controlsd",
  "plannerd": "selfdrive.controls.plannerd",
  "radard": "selfdrive.controls.radard",
  "dmonitoringd": "selfdrive.monitoring.dmonitoringd",
  "ubloxd": ("selfdrive/locationd", ["./ubloxd"]),
  "loggerd": ("selfdrive/loggerd", ["./loggerd"]),
  "logmessaged": "selfdrive.logmessaged",
  "locationd": "selfdrive.locationd.locationd",
  "tombstoned": "selfdrive.tombstoned",
  "logcatd": ("selfdrive/logcatd", ["./logcatd"]),
  "proclogd": ("selfdrive/proclogd", ["./proclogd"]),
  "boardd": ("selfdrive/boardd", ["./boardd"]),   # not used directly
  "pandad": "selfdrive.pandad",
  "ui": ("selfdrive/ui", ["./ui"]),
  "calibrationd": "selfdrive.locationd.calibrationd",
  "paramsd": "selfdrive.locationd.paramsd",
  "camerad": ("selfdrive/camerad", ["./camerad"]),
  "sensord": ("selfdrive/sensord", ["./sensord"]),
  "clocksd": ("selfdrive/clocksd", ["./clocksd"]),
  "gpsd": ("selfdrive/sensord", ["./gpsd"]),
  "updated": "selfdrive.updated",
  "dmonitoringmodeld": ("selfdrive/modeld", ["./dmonitoringmodeld"]),
  "modeld": ("selfdrive/modeld", ["./modeld"]),
  "mapd": ("selfdrive/mapd", ["./mapd.py"]),
  "rtshield": "selfdrive.rtshield",
}

daemon_processes = {
  "manage_athenad": ("selfdrive.athena.manage_athenad", "AthenadPid"),
}

running: Dict[str, Process] = {}
def get_running():
  return running

# due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
unkillable_processes = ['camerad']

# processes to end with SIGINT instead of SIGTERM
interrupt_processes: List[str] = []

# processes to end with SIGKILL instead of SIGTERM
kill_processes = ['sensord']

persistent_processes = [
  'thermald',
  'logmessaged',
  'ui',
  'uploader',
  'deleter',
]

if not PC:
  persistent_processes += [
    'updated',
    'logcatd',
    'tombstoned',
    'sensord',
  ]

car_started_processes = [
  'controlsd',
  'plannerd',
  'loggerd',
  'radard',
  'calibrationd',
  'paramsd',
  'camerad',
  'modeld',
  'proclogd',
  'ubloxd',
  'mapd',
  'locationd',
  'clocksd',
]

driver_view_processes = [
  'camerad',
  'dmonitoringd',
  'dmonitoringmodeld'
]
if traffic_lights:
  car_started_processes += [
    'trafficd',
    'traffic_manager',
  ]

if not PC or WEBCAM:
  car_started_processes += [
    'ubloxd',
    'dmonitoringd',
    'dmonitoringmodeld',
  ]

if EON:
  car_started_processes += [
    'gpsd',
    'rtshield',
  ]


def register_managed_process(name, desc, car_started=False):
  global managed_processes, car_started_processes, persistent_processes
  managed_processes[name] = desc
  if car_started:
    car_started_processes.append(name)
  else:
    persistent_processes.append(name)

# ****************** process management functions ******************
def nativelauncher(pargs, cwd):
  # exec the process
  os.chdir(cwd)

  # because when extracted from pex zips permissions get lost -_-
  os.chmod(pargs[0], 0o700)

  os.execvp(pargs[0], pargs)

def start_managed_process(name):
  if name in running or name not in managed_processes:
    return
  proc = managed_processes[name]
  if isinstance(proc, str):
    cloudlog.info("starting python %s" % proc)
    running[name] = Process(name=name, target=launcher, args=(proc,))
  else:
    pdir, pargs = proc
    cwd = os.path.join(BASEDIR, pdir)
    cloudlog.info("starting process %s" % name)
    running[name] = Process(name=name, target=nativelauncher, args=(pargs, cwd))
  running[name].start()

def start_daemon_process(name):
  params = Params()
  proc, pid_param = daemon_processes[name]
  pid = params.get(pid_param, encoding='utf-8')

  if pid is not None:
    try:
      os.kill(int(pid), 0)
      with open(f'/proc/{pid}/cmdline') as f:
        if proc in f.read():
          # daemon is running
          return
    except (OSError, FileNotFoundError):
      # process is dead
      pass

  cloudlog.info("starting daemon %s" % name)
  proc = subprocess.Popen(['python', '-m', proc],  # pylint: disable=subprocess-popen-preexec-fn
                          stdin=open('/dev/null', 'r'),
                          stdout=open('/dev/null', 'w'),
                          stderr=open('/dev/null', 'w'),
                          preexec_fn=os.setpgrp)

  params.put(pid_param, str(proc.pid))

def prepare_managed_process(p, build=False):
  proc = managed_processes[p]
  if isinstance(proc, str):
    # import this python
    cloudlog.info("preimporting %s" % proc)
    importlib.import_module(proc)
  elif os.path.isfile(os.path.join(BASEDIR, proc[0], "SConscript")) and build:
    # build this process
    cloudlog.info("building %s" % (proc,))
    try:
      subprocess.check_call(["scons", "u", "-j4", "."], cwd=os.path.join(BASEDIR, proc[0]))
    except subprocess.CalledProcessError:
      # clean and retry if the build failed
      cloudlog.warning("building %s failed, cleaning and retrying" % (proc, ))
      subprocess.check_call(["scons", "-u", "-c", "."], cwd=os.path.join(BASEDIR, proc[0]))
      subprocess.check_call(["scons", "-u", "-j4", "."], cwd=os.path.join(BASEDIR, proc[0]))


def join_process(process, timeout):
  # Process().join(timeout) will hang due to a python 3 bug: https://bugs.python.org/issue28382
  # We have to poll the exitcode instead
  t = time.time()
  while time.time() - t < timeout and process.exitcode is None:
    time.sleep(0.001)


def kill_managed_process(name):
  if name not in running or name not in managed_processes:
    return
  cloudlog.info("killing %s" % name)

  if running[name].exitcode is None:
    if name in interrupt_processes:
      os.kill(running[name].pid, signal.SIGINT)
    elif name in kill_processes:
      os.kill(running[name].pid, signal.SIGKILL)
    else:
      running[name].terminate()

    join_process(running[name], 5)

    if running[name].exitcode is None:
      if name in unkillable_processes:
        cloudlog.critical("unkillable process %s failed to exit! rebooting in 15 if it doesn't die" % name)
        join_process(running[name], 15)
        if running[name].exitcode is None:
          cloudlog.critical("unkillable process %s failed to die!" % name)
          os.system("date >> /data/unkillable_reboot")
          os.sync()
          HARDWARE.reboot()
          raise RuntimeError
      else:
        cloudlog.info("killing %s with SIGKILL" % name)
        os.kill(running[name].pid, signal.SIGKILL)
        running[name].join()

  cloudlog.info("%s is dead with %d" % (name, running[name].exitcode))
  del running[name]


def cleanup_all_processes(signal, frame):
  cloudlog.info("caught ctrl-c %s %s" % (signal, frame))

  if EON:
    pm_apply_packages('disable')

  for name in list(running.keys()):
    kill_managed_process(name)
  cloudlog.info("everything is dead")


def send_managed_process_signal(name, sig):
  if name not in running or name not in managed_processes or \
     running[name].exitcode is not None:
    return

  cloudlog.info(f"sending signal {sig} to {name}")
  os.kill(running[name].pid, sig)


# ****************** run loop ******************

def manager_init():
  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set dongle id
  reg_res = register()
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception("server registration failed")
  os.environ['DONGLE_ID'] = dongle_id

  if not dirty:
    os.environ['CLEAN'] = '1'

  cloudlog.bind_global(dongle_id=dongle_id, version=version, dirty=dirty, is_eon=True)
  crash.bind_user(id=dongle_id)
  crash.bind_extra(version=version, dirty=dirty, is_eon=True)

  os.umask(0)
  try:
    os.mkdir(ROOT, 0o777)
  except OSError:
    pass

  # ensure shared libraries are readable by apks
  if EON:
    os.chmod(BASEDIR, 0o755)
    os.chmod("/dev/shm", 0o777)
    os.chmod(os.path.join(BASEDIR, "cereal"), 0o755)
    os.chmod(os.path.join(BASEDIR, "cereal", "libmessaging_shared.so"), 0o755)

def manager_thread():

  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  EnableLogger = int(params.get('OpkrEnableLogger'))     

  if not EnableLogger:
    car_started_processes.remove( 'loggerd' )
    persistent_processes.remove( 'logmessaged' )
    persistent_processes.remove( 'uploader' )
    persistent_processes.remove( 'logcatd' )
    persistent_processes.remove( 'updated' )
    persistent_processes.remove( 'deleter' )
    persistent_processes.remove( 'tombstoned' )
  else:
    # save boot log
    subprocess.call(["./loggerd", "--bootlog"], cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))


  # start daemon processes
  for p in daemon_processes:
    start_daemon_process(p)

  # start persistent processes
  for p in persistent_processes:
    start_managed_process(p)

  # start offroad
  if EON:
    pm_apply_packages('enable')
    start_offroad()

  if os.getenv("NOBOARD") is None:
    start_managed_process("pandad")

  if os.getenv("BLOCK") is not None:
    for k in os.getenv("BLOCK").split(","):
      del managed_processes[k]

  started_prev = False
  logger_dead = False
  thermal_sock = messaging.sub_sock('thermal')

  while 1:
    msg = messaging.recv_sock(thermal_sock, wait=True)

    if msg.thermal.freeSpace < 0.05:
      logger_dead = True

    if msg.thermal.started:
      for p in car_started_processes:
        if p == "loggerd" and logger_dead:
          kill_managed_process(p)
        else:
          start_managed_process(p)
    else:
      logger_dead = False
      driver_view = params.get("IsDriverViewEnabled") == b"1"

      # TODO: refactor how manager manages processes
      for p in reversed(car_started_processes):
        if p not in driver_view_processes or not driver_view:
          kill_managed_process(p)

      for p in driver_view_processes:
        if driver_view:
          start_managed_process(p)
        else:
          kill_managed_process(p)

      # trigger an update after going offroad
      if started_prev:
        os.sync()
        send_managed_process_signal("updated", signal.SIGHUP)

    started_prev = msg.thermal.started

    # check the status of all processes, did any of them die?
    running_list = ["%s%s\u001b[0m" % ("\u001b[32m" if running[p].is_alive() else "\u001b[31m", p) for p in running]
    cloudlog.debug(' '.join(running_list))

    # Exit main loop when uninstall is needed
    if params.get("DoUninstall", encoding='utf8') == "1":
      break

def manager_prepare(spinner=None):
  # build all processes
  os.chdir(os.path.dirname(os.path.abspath(__file__)))

  process_cnt = len(managed_processes)
  loader_proc = []
  params = Params()
  spinner_text = "dashcam" if params.get("Passive")=="1" else "프로세스"
  for n,p in enumerate(managed_processes):
    if os.getenv("PREPAREONLY") is None:
      loader_proc.append(subprocess.Popen(["./spinner",
        "{0} 로딩: {1}/{2} {3}".format(spinner_text, n+1, process_cnt, p)],
        cwd=os.path.join(BASEDIR, "selfdrive", "ui", "spinner"),
        close_fds=True))
    prepare_managed_process(p)

  # end subprocesses here to stop screen flickering
  [loader_proc[pc].terminate() for pc in range(process_cnt) if loader_proc]

def main():
  params = Params()
  params.manager_start()

  default_params = [
    ("CommunityFeaturesToggle", "0"),
    ("CompletedTrainingVersion", "0"),
    ("IsRHD", "0"),
    ("IsMetric", "1"),
    ("RecordFront", "0"),
    ("HasAcceptedTerms", "0"),
    ("HasCompletedSetup", "0"),
    ("IsUploadRawEnabled", "1"),
    ("IsLdwEnabled", "1"),
    ("IsGeofenceEnabled", "-1"),
    ("LimitSetSpeed", "0"),
    ("LimitSetSpeedNeural", "0"),
    ("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')),
    ("OpenpilotEnabledToggle", "1"),
    ("LaneChangeEnabled", "1"),
    ("IsDriverViewEnabled", "0"),
    ("IsOpenpilotViewEnabled", "0"),
    ("OpkrAutoShutdown", "2"),
    ("OpkrAutoScreenOff", "0"),
    ("OpkrUIBrightness", "0"),
    ("OpkrEnableDriverMonitoring", "1"),
    ("OpkrEnableLogger", "0"),
    ("OpkrEnableGetoffAlert", "1"),
    ("OpkrAutoResume", "1"),
    ("OpkrVariableCruise", "0"),
    ("OpkrLaneChangeSpeed", "60"),
    ("OpkrAutoLaneChangeDelay", "0"),
    ("OpkrSteerAngleCorrection", "0"),
    ("PutPrebuiltOn", "0"),
    ("FingerprintIssuedFix", "0"),
    ("LdwsCarFix", "0"),
    ("LateralControlMethod", "2"),
    ("CruiseStatemodeSelInit", "1"),
    ("InnerLoopGain", "30"),
    ("OuterLoopGain", "20"),
    ("TimeConstant", "10"),
    ("ActuatorEffectiveness", "15"),
    ("Scale", "1750"),
    ("LqrKi", "10"),
    ("DcGain", "30"),
    ("IgnoreZone", "0"),
    ("PidKp", "20"),
    ("PidKi", "40"),
    ("PidKf", "5"),
    ("CameraOffsetAdj", "60"),
    ("SteerRatioAdj", "140"),
    ("SteerActuatorDelayAdj", "35"),
    ("SteerRateCostAdj", "45"),
    ("SteerLimitTimerAdj", "40"),
    ("TireStiffnessFactorAdj", "85"),
    ("SteerMaxAdj", "380"),
    ("SteerMaxBaseAdj", "275"),
    ("SteerDeltaUpAdj", "3"),
    ("SteerDeltaDownAdj", "7"),
    ("SteerMaxvAdj", "10"),
    ("OpkrBatteryChargingControl", "1"),
    ("OpkrBatteryChargingMin", "70"),
    ("OpkrBatteryChargingMax", "80"),
    ("OpkrUiOpen", "0"),
    ("OpkrDriveOpen", "0"),
    ("OpkrTuneOpen", "0"),
    ("OpkrControlOpen", "0"),
    ("LeftCurvOffsetAdj", "0"),
    ("RightCurvOffsetAdj", "0"),
    ("DebugUi1", "0"),
    ("DebugUi2", "0"),
    ("OpkrBlindSpotDetect", "1"),
    ("OpkrMaxAngleLimit", "90"),
    ("OpkrAutoResumeOption", "1"),
    ("OpkrAngleOffsetSelect", "0"),
    ("OpkrSpeedLimitOffset", "0"),
    ("LimitSetSpeedCamera", "0"),
    ("OpkrLiveSteerRatio", "0"),
    ("OpkrVariableSteerMax", "1"),
    ("OpkrVariableSteerDelta", "0"),
  ]

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # is this dashcam?
  if os.getenv("PASSIVE") is not None:
    params.put("Passive", str(int(os.getenv("PASSIVE"))))

  if params.get("Passive") is None:
    raise Exception("Passive must be set to continue")

  if EON:
    update_apks()
  manager_init()
  manager_prepare(spinner)
  spinner.close()

  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    crash.capture_exception()
  finally:
    cleanup_all_processes(None, None)

  if params.get("DoUninstall", encoding='utf8') == "1":
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()


if __name__ == "__main__":
  try:
    main()
  except Exception:
    add_logentries_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()
    process = subprocess.check_output(['git', 'pull'])
    os.system('reboot')

    raise

  # manual exit because we are forked
  sys.exit(0)
