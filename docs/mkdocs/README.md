# ClimbLab Manual

Author(s) and maintainer(s): [Space Robotics Lab.](http://www.astro.mech.tohoku.ac.jp/e/index.html) Climbing Robotics Team

* **Repo admin**: Warley Ribeiro (warley at dc.tohoku.ac.jp)

* **Team admin**: Kentaro Uno (unoken at tohoku.ac.jp)

[![srl-logo-original.jpg](./docs/img/media/srl-logo-original.jpg)](http://www.astro.mech.tohoku.ac.jp/e/index.html)![crt_color_logo_a_hi-reso.png](./docs/img/media/crt_color_logo_a_hi-reso.png)

The climbing simulator is being developed by the Climbing Robotics Team in [Space Robotics Laboratory](http://www.astro.mech.tohoku.ac.jp/e/index.html) at Tohoku University, Japan.

## Overview

This electrical manual has the following advantages:

☑ You can edit each chapter in the markdown style.

☑ MkDocs function configures the markdown style files as the one clickable manuals.

☑ You can download the manual as PDF easily.



## Usage

#### Requirements
We confirmed the code is working with:

* Python 2.7.17

* mkdocs, version 1.0.4 from /home/unoken/.local/lib/python2.7/site-packages/mkdocs (Python 2.7)

* ubuntu 18.04 LTS

#### Build and Run

* Install MkDocs with pip command if you do not have it on your Linux OS.


```
$ pip install mkdocs
```

* Change the directory of MkDocs and type the build command.

```
$ cd ~/bibucket/climblab/docs/mkdocs
$ mkdocs build
```

* Run the command to produce the interactive manual by the following command:

```
$ mkdocs serve
```

* click [http://127.0.0.1:8000/] to view the interactive manual in the browser.

