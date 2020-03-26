# Save results to a SQL database

## Requirements

***ODB 2.5.0***
> Note: 2.5.0 is a beta version, but it is required if you are using Ubuntu 18.04 LTS and
gcc 7.4.0, if you are using earlier ubuntu/gcc versions, proceed to the installation of ODB 2.4.0
below.

Follow the instructions [here](https://www.codesynthesis.com/products/odb/doc/install-build2.xhtml)
 to install build2 toolchain, odb compiler and runtime libraries.

Required runtime libraries are: libodb, libodb-sqlite/libodb-pgsql/libodb-mysql (depending on which
 database you want to use) and libodb-boost.

Remember to install the correct gcc plugin, for example (for gcc 7):
```
sudo apt-get update
sudo apt-get install gcc-7-plugin-dev
```
Additionally, if you want to use MYSQL database, you will need to build a not yet released version,
so instead of executing:
```
bpkg build libodb-mysql
bpkg install libodb-mysql
```
you will need to execute:
```
bpkg build libodb-mysql@https://git.codesynthesis.com/odb/libodb-mysql.git#fix-bind-decl
bpkg install libodb-mysql
```
After installing all the libraries make sure to add `/usr/local/lib` to your PATH:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

***ODB 2.4.0:***
All the downloads can be found [here](https://www.codesynthesis.com/products/odb/download.xhtml).

* Odb compiler
* Common Runtime Library: libodb-2.4.0
* Database Runtime Library: libodb-sqlite-2.4.0/libodb-mysql-2.4.0/libodb-pgsql-2.4.0
* Profile Libraries: libodb-boost-2.4.0, libodb-qt-2.4.0

Follow the [instructions](https://www.codesynthesis.com/products/odb/doc/install-unix.xhtml) to
install all the components.

## How to build and run

```
source ros2_install_path/setup.bash
mkdir -p perf_test_ws/src
cd perf_test_ws/src
git clone https://github.com/ApexAI/performance_test.git
cd ..
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DPERFORMANCE_TEST_ODB_SQLITE=ON
./install/performance_test/lib/performance_test/perf_test -c ROS2 -l log -t Array1k --max_runtime 10
```

The default name of the resulting database is "db_name", you can change it by using `--db_name`
argument in when executing performance_test. For MySQL or PostgreSQL databases, you can also specify `--db_user`,
`--db_password`, `--db_host` and `--db_port` to connect to your database.

In order to run the performance test with MySQL or PostgreSQL support, you can use
`-DPERFORMANCE_TEST_ODB_MYSQL=ON` or `-DPERFORMANCE_TEST_ODB_PGSQL=ON` instead of
`-DPERFORMANCE_TEST_ODB_SQLITE=ON` option.

The project supports [schema evolution](https://www.codesynthesis.com/products/odb/doc/manual.xhtml#13).
Every time you make a change in C++ code base which requires a schema evolution you need to:

1. Update the model version (`#pragma db model version`).
2. Commit the xml files with schema changes (`schema_changelog` folder).

> Note: The current `schema_changelog` is implemented for `MySQL` database. In order to use
other database, you need to start a new schema evolution:
    1. Remove the existing xml files (please do not commit these changes).
    2. Run the performance test with db model version 1 (specified in `experiment_configuration.hpp` file).

> All the necessary changes to add SQL database support to the performance_test tool were made by
following instructions from [ODB platform](https://www.codesynthesis.com/products/odb/). Please
refer to the [ODB manual](https://www.codesynthesis.com/products/odb/doc/odb-manual.pdf) for more information
 and implementation details.
