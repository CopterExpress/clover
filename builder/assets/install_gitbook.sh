#!/usr/bin/env bash

# GitBook CLI is deprecated, its installation is broken.
# This script fixes it until we stop using GitBook.

export NPM_CONFIG_UNSAFE_PERM=1

npm install gitbook-cli -g
gitbook fetch 3.2.3 && npm i npm@3.10.10 --prefix=~/.gitbook/versions/3.2.3/ # fixing https://travis-ci.org/github/CopterExpress/clover/jobs/766541125#L932
