Folder structure should follow:

```
datasets/
├── dataset_1/
│   ├── dataset_1.ds.yml - Dataset description file
│   ├── grasps/
│   │   ├── object_1.grasp.yml - Set of grasp candidates for object 1
│   │   │ ...
│   │   └── object_n.grasp.yml
│   ├── models/
│   │   ├── object_1/ - Folder with object 1 Gazebo model
│   │   │ ...
│   │   └── object_n/
│   └── rest/
│       ├── object_1.rest.yml - Set of rest poses for object 1
│       │ ...
│       └── object_n.rest.yml
└── dataset_2/
     ...
```