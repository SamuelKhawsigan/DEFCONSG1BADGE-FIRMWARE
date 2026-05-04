# dcsgonefirm

hey guys so this is the firmware for our badge thingy. 

### random notes
im still working on this so some stuff might be broken or marked with 'Dev'. if u see something wrong just fix it lol

### how to build it
u need esp-idf for this. v5.5.4
just run this in the terminal:
```
idf.py set-target esp32c6
idf.py build
idf.py flash
```
or just press the build button in your IDE if ur using one of those.

### where stuff is
so basically if u have a huge app put it in the components folder. if its just a small script or something just dump it in the `main/` folder. oh and if u need to save pictures or text files put them in `spiffs/`!


