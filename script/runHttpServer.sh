#!/bin/bash

source ../web/venv/bin/activate
cd ../web/server
python -m http.server 4444
