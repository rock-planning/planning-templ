# Configuration
The planner has a number of configuration options to influence the
search.
Within the C++ code the configuration options are read using the
Configuration class using [qxcfg](https://github.com/rock-tools/tools-qxcfg),
e.g.,

```
    configuration.getValueAs<double>("TransportNetwork/search/options/connectivity/timeout_in_s",20),
```

An example and the default configuration can be found in the
[test/data/configuration](../test/data/configuration) directory.

A configuration example is listed in the following:
```
<templ-configuration>
    <TransportNetwork>
        <logging>
            <basedir>/tmp</basedir>
        </logging>
        <use-transfer-location>false</use-transfer-location>
        <!-- default is false -->
        <intervals-nooverlap>false</intervals-nooverlap>
        <search>
            <options>
                <connectivity>
                    <interface-type>http://www.rock-robotics.org/2014/01/om-schema#ElectroMechanicalInterface</interface-type>
                    <timeout_in_s>20</timeout_in_s>
                </connectivity>
                <coalition-feasibility>
                    <timeout_in_s>1</timeout_in_s>
                </coalition-feasibility>
                <threads>1</threads>
                <cutoff>2</cutoff>
                <nogoods_limit>128</nogoods_limit>
                <computation_distance>120</computation_distance>
                <adaptive_computation_distance>40</adaptive_computation_distance>
                <epoch_timeout_in_s>60</epoch_timeout_in_s><!-- stop: when a single search ends and a restart should be triggered -->
                <total_timeout_in_s>900</total_timeout_in_s><!-- stop: when the total search ends -->
                <allow-flaws>false</allow-flaws>
                <model-usage>
                    <afc-decay>0.95</afc-decay>
                </model-usage>
                <role-usage>
                    <force-min>false</force-min><!-- enforce minimum role usage -->
                    <mobile>
                        <bounded>true</bounded><!-- if force min is deactivated then optionally use bounded with given bound -->
                        <bound-offset>2</bound-offset>
                    </mobile>
                    <immobile>
                        <bounded>true</bounded><!-- if force min is deactivated then optionally use bounded with given bound -->
                        <bound-offset>0</bound-offset>
                    </immobile>
                    <!-- todo: consider random upper bound -->
                    <afc-decay>0.95</afc-decay>
                </role-usage>
                <master-slave>false</master-slave><!-- allow to improve solution using a master-slave approach applying flaw resolvers -->
                <!-- default is false -->
                <hill-climbing>false</hill-climbing><!-- allow only increasingly better solutions, by constrain in master constrain function -->
                <timeline-brancher>
                    <afc-decay>0.95</afc-decay>
                    <supply-demand>false</supply-demand>
                </timeline-brancher>
                <lp>
                    <!-- CBC_SOLVER, CLP_SOLVER, SCIP_SOLVER or GLPK_SOLVER -->
                    <solver>CLP_SOLVER</solver>
                    <cache-solution>false</cache-solution>
                </lp>
                <cost-function>
                    <efficacy>
                        <weight>1.0</weight>
                    </efficacy>
                    <efficiency>
                        <weight>1.0</weight>
                    </efficiency>
                    <safety>
                        <weight>1.0</weight>
                    </safety>
                </cost-function>
            </options>
            <interactive>false</interactive>
        </search>
    </TransportNetwork>
</templ-configuration>
```

## General configuration options
### use-transfer-location
If this option is set to true, then an additional / extra so-called transfer location is added. It can be interpreted as an arbitrarily location exchange hub.

### intervals-noooverlap
If interval-nooverlap is defined, then all defined interval must not overlap.

### logging
 * basedir: The target directory for log files. TemPl creates a timestamp
   directory for each run of templ, which then contains a 'spec' directory
   containing the mission.xml and the configuration, and so-called session
   directories for each epoch of the planner

## Search
### interactive
If interactive is set to true then a user will be prompted to step through the planner.
This is mainly intended for debugging an gaining a general understanding of the internal working of the planner.

### options
| Option name | Default value | Description |
|-----|-------|-------|
|connectivity/interface-type| http://www.rock-robotics.org/2014/01/om-schema#ElectroMechanicalInterface |Pick the base interface type that should be considered to create composite system. This interface type has to be defined in the used ontology of the mission|
|connectivity/timeout_in_s| 20 |Testing of the connectivity suffers from combinatorial explosion and in worst case if no connection can be found - a exhaustive search has to be made. Hence, connectivity checking is limited by time timeout |
|coalition-feasibility | 1 | Similar to connectivity checking the validation of a feasible coalition is in worst case exhaustive and thus is limited by thie timeout |
| threads | 1| number of threads that can be used|
| cutoff  | 2 | Gecode CSP parameter: when to perform a cutoff |
| nogoods_limit | 128 | Gecode CSP parameter: limit the number of recorded nogoods |
| computation_distance | 120 | Gecode CSP parameter: variable distance after which a space will be recomputed|
| adaptive_computation_distance |40 | Gecode CSP parameter: |
| epoch_timeout_in_s| 60 | |
| total_timeout_in_s| 900 | |
| allow-flaws| false | |
| model-usage/afc-decay|0.95| |
| role-usage/afc-decay|0.95| |
| role-usage/force-min|false | |
| role-usage/mobile/bounded|true||
| role-usage/mobile/bound-offset|2|
| role-usage/immobile/bounded|true|
| role-usage/immobile/bound-offset|0|
| master-slave | false |allow to improve solution using a master-slave approach applying flaw resolvers|
| hill-climbing| false | allow only increasingly better solutions, by constrain in master constrain function|
| timeline-brancher/afc-decay| 0.95| |
| timeline-brancher/supply-demand|false| |
| lp/solver|CLP_SOLVER | CBC_SOLVER, CLP_SOLVER, SCIP_SOLVER or GLPK_SOLVER |
| lp/cache-solution|false ||
| cost-function/efficacy/weight|1.0||
| cost-function/effiency/weight|1.0||
| cost-function/safety/weight|1.0||


