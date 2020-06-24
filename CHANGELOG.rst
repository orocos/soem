^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soem
^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* remove package upgrade message
* Contributors: D2/EIN-gr Gruhler, Matthias

1.4.1000 (2020-06-19)
---------------------
* fix formatting of README
* remove stale bot
* change version number policy and bump to 1.4.1000
* bump cmake_minimum to 3.0.2
* Merge pull request `#33 <https://github.com/mgruhler/soem/issues/33>`_ from seanyen/windows
  [master] Enable Windows build.
* undef WIN32_LEAN_AND_MEAN instead of touching SOEM code.
* Enable Windows build.
* Contributors: Matthias Gruhler, seanyen

1.4.0 (2019-09-19)
------------------
* Merge pull request `#24 <https://github.com/mgruhler/soem/issues/24>`_ from mgruhler/upstream_soem_via_subtree
  Upstream soem release 1.4.0 via subtree
* explicitely export pthread dependency, reported in `#30 <https://github.com/mgruhler/soem/issues/30>`_
* install all headers in osal and oshw
* fixes `#29 <https://github.com/mgruhler/soem/issues/29>`_: provides issue templates
  provides three templates
  - bug report
  - feature request
  - question
* fixes `#28 <https://github.com/mgruhler/soem/issues/28>`_: adds note about ethercat_grant to README
* fix readme justification
* Merge branch 'master' into upstream_soem_via_subtree
* Merge pull request `#25 <https://github.com/mgruhler/soem/issues/25>`_ from mgruhler/upgrade_announcement
  add upgrade announcement
* add upgrade announcement
* change deprecation message
  * switch from type DEPRECATION to type <none>
  * minor formatting and typo fixes
* make new integration backwards compatible
  - add deprecation warning of workaround to CMakeLists.txt
  - change order of include directories in cfg extras file
* restore version number and relevant package.xml contents
* update docs
  * fix description in package.xml
  * update README
* setup ROS build instructions for upstream SOEM as subtree
  - add new package.xml and CMakeLists.txt
  - add -fPIC definition and comment ROS specific "hacks"
  - copy headers to develspace instead of src space
  - provide cmake extras file to set include directories
  - fix cmake_minimum_required for using add_compile_options
  - ignore catkin_lint errors for SOEM CMakeLists.txt
* Merge commit 'd471a3878b20d650a6d80a487625324392a23906' as 'SOEM'
* Squashed 'SOEM/' content from commit abbf0d4
  git-subtree-dir: SOEM
  git-subtree-split: abbf0d42e38d6cfbaa4c1e9e8e07ace651c386fd
* remove old SOEM state
* State release into ROS melodic in README.md
* Add close comment and extend daysUntilClose in stale.yaml
* Update README.md: clarify Usage section
  add note to either use from apt or followed development section
* add stale.yml for Stale Probot
* add README.md:
  * clarifying what this package is about
  * adding sections about installation, usage and development
* update description in package.xml
  - more explicitely explain what this package is about
  - link more directly to upstream by noting what this is actually based upon
* update description and URLs in package.xml
* update maintainer

1.3.0 (2015-01-26)
-------------------
* Update soem version number
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* cmake: remove erroneous linking to pq
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* cmake: use non-capitalised project name in cmake config file
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* Merge pull request `#3 <https://github.com/mgruhler/soem/issues/3>`_ from meyerj/cmake-fixes
  Some more cmake fixes required for installation
* osal: added missing header installation
* cmake: fixed exported cmake config
* Merge pull request `#1 <https://github.com/mgruhler/soem/issues/1>`_ from meyerj/patch-1
  Fixed header installation in oshw
* Fixed header installation in oshw
* add missing CMakeLists.txt for oshw dir
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* finalized cmake build system for 1.3.0
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* Import upstream release 1.3.0
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* move src to soem directory to prepare 1.3.0 upgrade
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* Added install rule for package.xml
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>

1.2.5 (2013-03-06)
-------------------
* add cmake build system
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* Upstream SOEM 1.2.5
  Signed-off-by: Ruben Smits <ruben.smits@intermodalics.eu>
* Contributors: Johannes Meyer, Matthias Gruhler, Ruben Smits
