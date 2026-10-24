# Field data and asset attribution

The source presentation identifies the University of Bonn MuST-C dataset as the field-data source. MuST-C provides georeferenced measurements from aerial and ground platforms, including RGB, LiDAR and multispectral sensing.

- Dataset: https://www.ipb.uni-bonn.de/data/MuST-C/
- Processing tools: https://github.com/PRBonn/MuST-C
- Paper: Chong et al., *The Multi-Sensor and Multi-Temporal Dataset of Multiple Crops for In-Field Phenotyping and Monitoring*, Scientific Data 13 (2026), https://doi.org/10.1038/s41597-025-06462-y
- Dataset license: Creative Commons Attribution 4.0, https://creativecommons.org/licenses/by/4.0/

The field-generation pipeline adapts geographic plot geometry, terrain imagery and point-cloud-derived field models into simulation assets. These are modified representations, not unmodified sensor measurements. Exact source-date and transformation metadata are not available for every mesh; retain that limitation when reusing them.

Raw orthophoto/raster archives are deliberately omitted. Prepared route/geometry outputs and retained simulation assets enable inspecting the ground-robot workflow. To regenerate the GIS pipeline, download the relevant source data from the dataset provider, inspect each script's input arguments and paths, and preserve attribution with any further derivatives.

The presentation describes a field approximately 174.6 × 63.9 m, a 242-plot geographic layout, and an 80-plot UAV scan area. These are project context; the default `soil.sdf` is a selected scene and should not be assumed to contain a complete reconstruction of every plot.
