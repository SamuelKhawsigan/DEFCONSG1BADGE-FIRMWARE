# dcsgonefirm

# Notes

As this repo is still being worked on, some things in this Readme are marked 
with 'Dev:'. Feel free to change this if needed.

# Building this

You need ESP-IDF, any modern version should work. (Dev: Change this if you 
require a more specific version)

Command-line:

	idf.py set-target esp32c6
	idf.py build
	idf.py flash

Or do the equivalent in your IDE.

# Structure

In general, large applications should be included as components in this repo. Smaller
things that don't warrant their own component can live in the main/ directory. If
you need data in the form of a file, feel free to dump it in the spiffs/ folder.

# Developer notes

Dev: Note that the sdkconfig is *not* included in this repo and should not be checked 
in (as it gets overwritten when you run 'idf.py set-target'). If you need something
changed, run menuconfig and use 'save minimal config', then copy ``build/defconfig``
to ``sdkconfig.defaults``.

If you want to change this code, please fork it first, make your changes, then
create a merge request in the main repo. If you get the OK from at least one other
team member, you can go ahead and merge it into main.
