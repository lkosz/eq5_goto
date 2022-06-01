class eq_telescope::eq_telescope_ra(){

  $urzadzenie = 'eq1_ra'

  file {"/opt/teleskop/${urzadzenie}.py":
    ensure  => file,
    mode    => '0744',
    content => template("eq_telescope/${urzadzenie}.py")
  }

  file{'/etc/systemd/system/teleskop_eq1_ra.service':
    ensure => file,
    content => template('eq_telescope/service.erb'),
  }~>
  service{'teleskop_eq1_ra':
    ensure   => running,
    enable   => true,
    provider => 'systemd',
  }

}
