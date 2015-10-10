#!/bin/bash -i

# sets up apache and webpage configuration

source $HOME/.bashrc

rospd usma_remote_interface/webpage

sudo apt-get update
sudo apt-get install apache2

echo "
<VirtualHost *:80>
	ServerAdmin webmaster@localhost
	DocumentRoot $(pwd)
  <Directory $(pwd)>
    Require all granted
  </Directory>

	ErrorLog"' ${APACHE_LOG_DIR}'"/error.log
	CustomLog"' ${APACHE_LOG_DIR}'"/access.log combined

</VirtualHost>
" >> tmp.conf

sudo rm /etc/apache2/sites-enabled/*
sudo mv tmp.conf /etc/apache2/sites-available/usma_remote_interface.conf
sudo ln -s /etc/apache2/sites-available/usma_remote_interface.conf /etc/apache2/sites-enabled/usma_remote_interface.conf

sudo service apache2 restart

rospd
