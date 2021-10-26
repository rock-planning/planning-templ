# Configuration
The planner has a number of configuration options to influence the
search.
Within the C++ code the configuration options are read using the
Configuration class using [qxcfg](https://github.com/rock-tools/tools-qxcfg),
e.g.,

```
    configuration.getValueAs<double>("TransportNetwork/search/options/connectivity/timeout_in_s",20),
```

An example / the default configuration can be found in the
[test/data/configuration](../test/data/configuration) directory.

Note that the used IRIs are resolvable online, but are only defined in the
ontologies of the dependency
[knowledge_reasoning/moreorg](http://github.com/rock-knowledge-reasoning/knowledge-reasoning-moreorg).
The ontologies will be resolved locally once moreorg has been installed and the
path variable OWLAPI_ONTOLOGIES_PATH has been set to the ontologies folder (if
you are bootstrapping with autoproj this is already done).

A configuration example is listed in the following.
Note that some of the options adjust
the search behavior of [Gecode](http://www.gecode.org) which is used as constraint
solver by TemPl.
These internal options are best described in the
official documentation [Modeling and Programming with Gecode](https://www.gecode.org/doc-latest/MPG.pdf) and we refer to the
corresponding chapter via MPG Chapter XXX in the following if needed, e.g., MPG
Chapter 9.3.1 'Search options'.

In addition to the Gecode documentation it is recommended to check Chapter 4.3 in the [PhD Thesis](http://nbn-resolving.de/urn:nbn:de:gbv:46-00107698-18)
to check for details:
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
                <cutoff>2</cutoff><!-- Gecode option: cutoff limit for the restart-based meta search engine MPG Chapter 9.4.2 Cutoff generator-->
                <nogoods_limit>128</nogoods_limit> <!-- Gecode option: no-goods from restarts MPG Chapter 9.2 No-goods from restarts -->
                <computation_distance>120</computation_distance> <!-- Gecode option for recomputation of solutions MPG Chapter 42 'Recomputation' -->
                <adaptive_computation_distance>40</adaptive_computation_distance><!-- Gecode option for recomputation of solutions MPG Chapter 42 'Recomputation' -->
                <epoch_timeout_in_s>60</epoch_timeout_in_s><!-- stop: when a single search ends and a restart should be triggered -->
                <total_timeout_in_s>900</total_timeout_in_s><!-- stop: when the total search ends -->
                <allow-flaws>false</allow-flaws>
                <model-usage><!-- solver for models: adapt internal gecode branching -->
                    <afc-decay>0.95</afc-decay>
                </model-usage>
                <role-usage><!-- adapt branching for roles (as model instances) see [Chapter 4.3 in Thesis](https://media.suub.uni-bremen.de/handle/elib/1674): adapt internal gecode branching -->
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
If interval-nooverlap is defined, then all defined intervals must not overlap.

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
| epoch_timeout_in_s| 60 | maximum time for internal epoch |
| total_timeout_in_s| 900 | maximum planning runtime in seconds |
| allow-flaws| false | allow incomplete solutions |
| model-usage/afc-decay|0.95| Accumulated Failure Count Decay, to influence variable selection|
| role-usage/afc-decay|0.95| Accumulated Failure Count Decay, to influence variable selection |
| role-usage/force-min|false | enforce minimal setup |
| role-usage/mobile/bounded|true| use bound offset for mobile systems |
| role-usage/mobile/bound-offset|2| maximum offset from minimal required (mobile) systems |
| role-usage/immobile/bounded|true| use bound offset for immobile systems |
| role-usage/immobile/bound-offset|0| maximum offset from minimal required (immobile) systems) |
| master-slave | false |allow to improve solution using a master-slave approach applying flaw resolvers|
| hill-climbing| false | allow only increasingly better solutions, by constrain in master constrain function|
| timeline-brancher/afc-decay| 0.95| Accumulated Failure Count Decay, to influence variable selection|
| timeline-brancher/supply-demand|false| Enable usage of explicit supply-demand computation (heavily affects performance, experimental)|
| lp/solver|CLP_SOLVER | CBC_SOLVER, CLP_SOLVER, SCIP_SOLVER or GLPK_SOLVER |
| lp/cache-solution|false | If true, LP Solution are cached to avoid recomputation|
| cost-function/efficacy/weight|1.0| Balancing factor for the cost function|
| cost-function/effiency/weight|1.0| Balancing factor for the cost function|
| cost-function/safety/weight|1.0| Balancing factor for the cost function|


