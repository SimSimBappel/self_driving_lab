# Postgresql Services

## System dependencies
```
sudo apt-get install -y libpqxx-dev libpqxx-6.4
```

## Testing services
Please see docstrings in source code or .srv and .msg files for detailed compositions of services. 

### Add chemical
This functionality allows the addition of chemicals to the database
```
ros2 service call /add_chemical pgsql_interfaces/srv/AddChemical "{name: 'natrium_chloride', formula: 'NaCL', safety_level: 0}"
```
### Add workstation
This functionality allows the addition of workstations to the database
```
ros2 service call /add_workstation pgsql_interfaces/srv/AddWorkstation "{name: 'stirring', type: 'workstation'}"
```

### Upsert chemical location 
This functionality allows upsertion of chemical locations to the database, if location_id passed the location is updated, if not a new location is inserted. 

Before this step an actual chemical should be added (this is searched in 'chemical' table)
```
ros2 service call /upsert_chemical_location pgsql_interfaces/srv/UpsertChemicalLocation "{id: 1, name: 'natrium_chloride', formula: 'NaCL', location_base: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, location_hand: {position: {x: 4.0, y: 5.0, z: 6.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, location_id: 1}"

ros2 service call /upsert_chemical_location pgsql_interfaces/srv/UpsertChemicalLocation "{id: 1, name: 'natrium_chloride', formula: 'NaCL', location_base: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, location_hand: {position: {x: 4.0, y: 5.0, z: 6.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Upsert workstation location 
This functionality allows upsertion of workstation locations to the database, if location_id passed the location is updated, if not a new location is inserted. 

Before this step an actual workstation should be added (this is searched in 'workstation' table)
```
ros2 service call /upsert_workstation_location pgsql_interfaces/srv/UpsertWorkstationLocation "{id: 1, name: '', type: '', location_base: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, location_hand: {position: {x: 4.0, y: 5.0, z: 6.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, location_id: 1}"

ros2 service call /upsert_workstation_location pgsql_interfaces/srv/UpsertWorkstationLocation "{id: 1, name: '', type: '', location_base: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, location_hand: {position: {x: 4.0, y: 5.0, z: 6.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```


### Get all chemical or workstation locations (two services)
This functionality returns all chemical or workstation locations as a list, as one chemical or workstation can be present in several places. 

```
ros2 service call /get_all_chemical_locations pgsql_interfaces/srv/GetAllChemicalLocations "{id: 1, name: '', formula: ''}"
ros2 service call /get_all_workstation_locations pgsql_interfaces/srv/GetAllWorkstationLocations "{id: 1, name: '', type: ''}"
```

### Remove workstation or chemical (two services)
In the first case the actual workstation is removed with locational cascade removal of all locations. 
```
ros2 service call /remove_workstation pgsql_interfaces/srv/RemoveWorkstation "{workstation_id: 1}"
```

For a specific location removal, pass the location_id to the same service
```
ros2 service call /remove_chemical pgsql_interfaces/srv/RemoveChemical "{chemical_id: 1, location_id: ''}"
```





