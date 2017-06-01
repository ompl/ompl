from fabric.api import *

# default set of hosts
env.hosts = [
    'megamedes.cs.rice.edu',
    'styx.cs.rice.edu',
    'oceanus.cs.rice.edu',
    'tethys.cs.rice.edu',
    'pallas.cs.rice.edu'
]
# run commands on other hosts like so:
#   fab -P update:hosts=dione.cs.rice.edu

env.user = 'zak'
env.key_filename = '/home/zak/.ssh/id_rsa'

def install_gitremotehg():
    run('mkdir ~/bin')
    # run('echo "export PATH=$PATH:/home/zak/bin" >> ~/.bashrc')
    run('wget https://raw.github.com/felipec/git-remote-hg/master/git-remote-hg -O ~/bin/git-remote-hg')
    run('chmod +x ~/bin/git-remote-hg')

def make_key():
    run('ssh-keygen -f /home/zak/.ssh/id_rsa')
    run('ssh-copy-id hera.cs.rice.edu')

@runs_once
def deploy():
    local('git pull origin branches/constraints')
    local('git push -v origin branches/constraints:refs/heads/branches/constraints')

@parallel
def install():
    run('rm -rf ~/ompl')
    run('PATH=$PATH:/home/zak/bin git clone "hg::ssh://zak@hera.cs.rice.edu://data/virtual/hg/ompl/"')

    with cd('~/ompl'):
        run('git gc --aggressive')
        run('git checkout branches/constraints')
        run('mkdir -p build/release')
        with cd('~/ompl/build/release'):
            run('cmake ../..')
            run('make -j8')

@parallel
def update():
    with cd('~/ompl'):
        run('PATH=$PATH:/home/zak/bin git pull')
        with cd('~/ompl/build/release'):
            run('cmake ../..')
            run('make -j8')
