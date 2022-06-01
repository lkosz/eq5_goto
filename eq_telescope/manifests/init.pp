class eq_telescope(
  Enum['eq3', 'eq2','eq1'] $urzadzenie,
){

  $packages = [
    'python3-cryptography',
    'gcc',
    'python3-dev',
    'python3-pip',
    'libatlas-base-dev',
    'python3-pigpio',
  ]

  package{['adafruit-circuitpython-mcp230xx', 'adafruit-circuitpython-ads1x15', 'skyfield', 'astropy']:
    provider => 'pip3',
  }

  zxa_package{ $packages :}->

  file {'/opt/teleskop':
    ensure => directory,
  }

  file {"/opt/teleskop/${urzadzenie}.py":
    ensure  => file,
    mode    => '0744',
    content => template("eq_telescope/${urzadzenie}.py")
  }

  file{'/lib/systemd/system/pigpiod.service':
    ensure => file,
    owner  => 'root',
    group  => 'root',
    mode   => '0644',
    source => 'puppet:///modules/eq_telescope/pigpiod.service',
    notify => Service['pigpiod'],
  }
  service{'pigpiod':
    ensure  => running,
    enable  => true,
    require => File['/lib/systemd/system/pigpiod.service'],
  }

  file{'/etc/systemd/system/teleskop.service':
    ensure => file,
    content => template('eq_telescope/service.erb'),
  }~>
  service{'teleskop':
    ensure   => running,
    enable   => true,
    provider => 'systemd',
  }

  firewall { '0300 traffic':
    chain         => 'INPUT',
    proto         => 'tcp',
    source        => <IP>,
    dport         => '80',
    action        => 'accept',
  }

}
