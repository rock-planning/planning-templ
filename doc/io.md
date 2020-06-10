## The Mission Description Format

The main mission description can be serialized as xml document.
The following sections describe the available tags describes the current format
using the [example file](examples/0_intro.xml)

### The Root Node
The root node's tag is 'mission' and since the document is valid xml, namespaces can be used to compact the representation of IRIs
(International Resource Identifier).

```
<mission xmlns="http://www.rock-robotics.org/missions#"
    xmlns:om="http://www.rock-robotics.org/2014/01/om-schema#"
    xmlns:xsd="http://www.w3.org/2001/XMLSchema#"
>
```

### Metadata
The 'name' and 'description' tag are mainly informal, and have no effect on the
planning part.
```
    <name>Immobile and mobile system deployment</name>
    <description>
        Immobile system needs a transport, but the mobile system is not dedicated to transport the 
        device
    </description>
```

### Organization Model
If you have a valid organization model installed - typically the TransTerrA
model will automatically be installed, through this library's dependency on the
library multiagent/organization model, then you have to reference it via its
IRI.
This allows then to use the agent models and properties that are defined within
the respective organization model, for further usage as 'model' or 'property'
for describing resource requirements and constraints.

```
    <organization_model>http://www.rock-robotics.org/2015/12/projects/TransTerrA</organization_model>
```

An 'overrides' section allows to set a numeric property for a particular model,
without manipulating the organization model directly.

```
<overrides>
  <override>
    <subject>http://www.rock-robotics.org/2014/01/om-schema#Sherpa</subject>
    <property>http://www.rock-robotics.org/2014/01/om-schema#transportCapacity</property>
    <value>12</value>
  </override>
</overrides>
```

### Available resources
Each mission has a limited fleet of agents that can be used for planning
purposes.
The fleet is specified in the 'resources' section and by defining the 'maxCardinality', i.e. maximum number
of agents of a particular model that can be used for this mission.
This does not mean that all available agents have to be used, a lesser number is
possible, but as 'max' implies, not more.
```
    <resources>
        <resource>
            <model>http://www.rock-robotics.org/2014/01/om-schema#Sherpa</model>
            <maxCardinality>1</maxCardinality>
        </resource>
        <resource>
            <model>http://www.rock-robotics.org/2014/01/om-schema#Payload</model>
            <maxCardinality>3</maxCardinality>
        </resource>
    </resources>
```
Note, that how an actual model looks like, what properties it has etc. that is all part of
the design of the referenced organization model.

### Constants
The following 'constants' are available for definition:

* location

### Location
A location can be specified in two ways: Cartesian (x,y,z) or using Longitude
Latitude:

The Cartesian representation is straight forward:
```
    <location>
        <id>l0</id>
        <x>0</x>
        <y>0</y>
        <z>0</z>
    </location>
```

The Longitude-Latitude representation requires the definition of a 'radius',
here, moon and earth, so that a projection to Cartesian coordinates can be made.
Note, that internally, a location will only be handled with Cartesian coordinates.

```
    <location>
        <id>l0</id>
        <radius>moon</radius>
        <latitude>-83.82009</latitude>
        <longitude>87.53932</longitude>
    </location>
```

### Timepoints
Although timepoints are constants, the library currently only uses an implicit
definition of them, i.e., they are can be used in the requirements or
constraints section without explicitly declaring them in the constant section.
Note, that this will be a subject to change.

## Spatio Temporal Requirements

Spatio Temporal Requirements (STRs) are collected with the 'requirements'
section and are represented as follows:

```
    <requirement id='0'>
        <spatial-requirement>
            <location>
                <id>l0</id>
            </location>
        </spatial-requirement>
        <temporal-requirement>
            <from>t0</from>
            <to>t1</to>
        </temporal-requirement>
        <resource-requirement>
            <resource>
                <model>http://www.rock-robotics.org/2014/01/om-schema#Sherpa</model>
                <minCardinality>1</minCardinality>
            </resource>
            <resource>
                <model>http://www.rock-robotics.org/2014/01/om-schema#Payload</model>
                <minCardinality>2</minCardinality>
            </resource>
        </resource-requirement>
    </requirement>
```
The spatial-requirement and temporal requirements in combination define a
space-interval tuple, where the location has to be defined in the constants
section.
The resource requirement collect minimum cardinality constraints, which are
specified for each model - only model that have been defined in the resources
section can be used.

## Constraints
Three types of constraints exist:

 * temporal (quantitative)
 * temporal (qualitative)
 * model

### Qualitative temporal constraints
Qualitative temporal constraints can be specified with using the following tags
representing the relations:

 * greaterThan
 * lessThan
 * equals
 * greaterOrEqual
 * lessOrEqual
 * distinct

The attributes lval and rval define the relation arguments.

### Quantitative temporal constraints
Quantitative temporal constraints are specified with the following syntax:
```
    <duration min="50" max="100">
            <from>t2</from>
            <to>t5</to>
    </duration>
```
where the min and max attributes define the lower and upper time bounds
respectively (in seconds), for the time interval [from,to].

### Model constraints:
The following model constraints are available:

|Type          | Description   |
|---|---|
| min          |   minimum cardinality   |
| max          |   maximum cardinality   |
over an interval)  |
| min-access   |   minimum access to a location (always or temporally bounded) |
| max-access   |   maximum access to a location (always or temporally bounded) |
| min-distinct |   minimum number of distinct model instances |
| max-distinct |   maximum number of distinct model instances |
| all-distinct |   all agent models have to be distinct  |
| min-equal    |   minimum number of equal model instances     |
| max-equal    |   maximum number of equal model instances     |
| all-equal    |   all model instances have to be the same     |
| min-function |   minimum function requirement    |
| max-function |   maximum function requirement     |
| min-property |   minimum value for a given property for an agent model     |
| max-property |   maximum value for a given property for an agent model     |

The constraint can apply to to existing requirements and their corresponding
spatio-temporal qualification (location, [from, to]).
Minimum and maximum access constraints can also refer only to the location,
which implies a constraint with a spatio-temporal qualification (location,
[timehorizon_start, timehorizon_end]), where timehorizon_start is the start of
the mission and timehorizon_end.
In this case the constraint applies to the timeinterval of the full mission.
A corresponding min access constraint can be interpreted such that a minimum of one 
model instance has to be present on this location of the full mission, e.g., to
guarantee an uninterrupted service, and max at most one can be
present at any time, e.g., which might be the result of size constraints.

Examples:
```
        <!-- location access constraint -->
        <min-access>
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>l0</requirements>
        </min-access>
        <max-access>
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>l0,1</requirements>
        </all-distinct>
        <!-- unification constraint -->
        <all-distinct>
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </all-distinct>
        <min-distinct value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </min-distinct>
        <max-distinct value="2"> <!-- that would equal all-distinct -->
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </max-distinct>
        <all-equal>
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </all-equal>
        <min-equal value="1">
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </min-equal>
        <max-equal value="1">
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </max-equal>

        <!-- function: number of distinct systems to be available -->
        <max-function value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
        </max-function>
        <min-function value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
        </min-function>

        <!-- when general properties makes sense -->
        <max-property value="1000">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
            <property>http://www.rock-robotics.org/2014/01/om-schema#mass</property>
        </max-property>
        <min-property value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
            <property>http://www.rock-robotics.org/2014/01/om-schema#transportCapacity</property>
        </min-property>

        <!-- min/max model cardinalities are part of the fluent time resource
             representation -->
```

