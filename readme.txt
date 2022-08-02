Instructions to install project:

Install acados:
https://docs.acados.org/installation/index.html

Install acados in project:
pip3 install -e <dir2acados>/acados/interfaces/acados_template
e.g.: pip3 install -e </home/rudolf/Programs>/acados/interfaces/acados_template

Install vehiclegym in project:
pip3 install -e <dir2vehiclegym>/vehiclegym/
e.g.: pip3 install -e /home/rudolf/PycharmProjects/vehiclegym

Add acados library path to variables:
https://docs.acados.org/python_interface/index.html
e.g. in pycharm Run->Edit Configuration->Environment Variables:
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/rudolf/Programs/acados/lib