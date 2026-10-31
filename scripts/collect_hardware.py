#!/usr/bin/env python3
"""Write results/hardware.json: the machine and software versions the results were produced on.
Everything is read from the running system (run it in the same container as the evaluation)."""
import datetime
import json
import os
import platform
import re
import shutil
import subprocess
import sys


def run(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception:
        return ''


def main():
    results = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('NAV_RESULTS_DIR', 'results')
    cpu_model = ''
    cores = os.cpu_count()
    with open('/proc/cpuinfo') as fh:
        for line in fh:
            if line.startswith('model name'):
                cpu_model = line.split(':', 1)[1].strip()
                break
    mem_kb = 0
    with open('/proc/meminfo') as fh:
        for line in fh:
            if line.startswith('MemTotal'):
                mem_kb = int(re.findall(r'\d+', line)[0])
                break
    lscpu = run('lscpu')
    hv = re.search(r'Hypervisor vendor:\s*(.+)', lscpu)
    gz = run('gzserver --version 2>&1 | head -1')
    info = {
        'generated_utc': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'cpu_model': cpu_model,
        'cpu_logical_cores': cores,
        'memory_gib': round(mem_kb / 1024 / 1024, 2),
        'hypervisor': hv.group(1).strip() if hv else '',
        'gpu': 'none used (headless gzserver, CPU ray sensor)',
        'kernel': platform.release(),
        'os_in_container': run("grep PRETTY_NAME /etc/os-release | cut -d= -f2 | tr -d '\"'"),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'gazebo': gz,
        'gcc': run('g++ --version | head -1'),
        'cmake': run('cmake --version | head -1'),
        'python': platform.python_version(),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', 'default (rmw_fastrtps_cpp)'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
        'docker_container': os.path.exists('/.dockerenv'),
        'git_commit': run('git -C "%s" rev-parse --short HEAD' % os.path.dirname(os.path.abspath(__file__))),
        'note': 'Simulated in Gazebo Classic; real-time factor is capped at 1.0 by world physics settings.',
    }
    os.makedirs(results, exist_ok=True)
    with open(os.path.join(results, 'hardware.json'), 'w') as fh:
        json.dump(info, fh, indent=2)
        fh.write('\n')
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
