#!/bin/bash
BuildVersion="${bamboo_VersionBase}${bamboo_buildNumber}"

echo "Tagging build ${BuildVersion}"

git tag -a -m "Tagging Jenkins build ${BUILD_URL} as ${BuildVersion}" "$BuildVersion"
git push --tags
