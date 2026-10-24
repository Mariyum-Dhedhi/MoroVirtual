# Viz_shapefiles

## Python virtual environment

This project uses a local virtual environment in `.venv`.

### Create the environment

```bash
/usr/bin/python3 -m venv .venv
```

### Activate the environment

Linux/macOS:

```bash
source .venv/bin/activate
```

Windows (PowerShell):

```powershell
.\.venv\Scripts\Activate.ps1
```

### Install dependencies

```bash
pip install -r requirements.txt
```

## Plot orthophoto with shapefile overlay

Run the script below to load the orthophoto and shapefile, then generate an overlay figure:

```bash
python scripts/plot_orthophoto_with_shapefile.py
```

The script saves the figure to `outputs/orthophoto_with_shapefile.png` and also opens a plot window.

If the shapefile uses geographic coordinates (longitude/latitude, EPSG:4326) and the orthophoto uses a projected CRS, the script automatically reprojects the shapefile to the orthophoto CRS before plotting.

Useful options:

```bash
# Save only (no plot window)
python scripts/plot_orthophoto_with_shapefile.py --no-show

# Use custom input/output paths
python scripts/plot_orthophoto_with_shapefile.py \
	--orthophoto orthophoto/230607_ortho.tif \
	--shapefile shapefiles/2023/PhenorobField_2023_lla.shp \
	--output outputs/my_overlay.png
```

### Deactivate when finished

```bash
deactivate
```
