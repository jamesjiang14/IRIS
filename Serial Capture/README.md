# Serial Capture + Image Generation

The contents of this folder include python scripts for receiving messages + image data sent by MAX78000 Feather Board (from CNN folder)

You must create a python virtual environment to run the scripts and install all requirements in `requirements.txt` to run the script. 

```python -m venv venv```

```.\venv\Scripts\activate.bat```

```pip install -r requirements.txt```

To run the `grab_image_hex.py` script (replace `COM4` with the COM port that the feather board is using)

```python grab_image_hex.py COM4```


