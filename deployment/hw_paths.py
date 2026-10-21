from pathlib import Path
import os
import shlex

def runtime_root():return Path(os.environ.get('SHAOSHAO_HW_RUNTIME',str(Path.home()/'ccc/shaoshao-hw-runtime')))

def setup_environment(root):
    return 'source /opt/ros/jazzy/setup.bash\nsource '+shlex.quote(str(Path(root)/'mapping_install/setup.bash'))+'\n'
