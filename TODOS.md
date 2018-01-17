# Completed Tasks
  - refactored constraint integration
  - play a graph: visualize incremental build up of a graph, i.e. visualize BFS
    or DFS [DONE for BFS: 2017-11-27]
    algorithm
  - updated PDDLExporter [2017-12-19]
  - gui
   - added ColorChooser ( QColorDialog cannot be used with the Delegate
    mechanism, therefore had to go a different route)
   - add setting of edge path style via QSettings
  - organization model
   - added Analyser infrastructure with Heuristics,StatusSample,AtomicAgent and
     Agent [2017-12-22]
   - added EnergyProviderPolicy: to define how to compute energy reduction for
     all power providers [2018-01-06]
      - since an individual atomic agent might not even have a power source, a
        policy has to define which atomic agent has to contribute to the power
        (or later perform a flow-based optimization on top of the solution, e.g.
        to allow to optimally design the power flow)
       - so for each 'agent' -- map energy consumption equally, but relatively to
         the overall capacity  --> this can be a source of investigation (allow
         to shift power between the power supplies)
    - update facets with model pool (rules) [done]
   - added TransportProviderPolicy: to define which system to use provider the
     'core' transport functionality of an agent
   - computation of reconfiguration cost / provide a time estimate [2018-01-09]
  - templ::Role moved to organization_model::AtomicAgent
  - gui: updated recent files menu

# Todos Backlog
## Priorities
 0. computing of a solution
    - general mechanism for additional constraints, e.g., meta constraints which are lazily translated into low level gecode constraints
    - construction heuristic
 0. analysing a solution
    - organization model analyser
       - identify active systems (primary required functionality, backup (kind of backup:
         hotswap, ....), active power supply/battery to draw from)
       - compute time/cost for wait/operation ( done for transfer/reconfiguration)
 0. optimizing a solution
 0. executing a solution

## Search
 - general mechanism for additional constraints, e.g., meta constraints
    [ongoing]
    which are lazily translated into low level gecode constraints

  - general representation for a solution:
    - a la chronicle/timelines, spacetime::network
     - spacetime-network: t0-l0 -- t1-l0: constraint from fluent time resource
       (functionality && resource)
    - local reconfiguration:
      - input commodities and transport -- local need -- output commodities and
        transport
    - time (quantified)

  - multi-commodity flow group for the same items - unless there is a
  constrained to split them?
    - -> actually temporal / parallel use will be prevented by multicommodity flow
  anyhow
    - -> so it comes down to selecting multi-robot team
    - -> allows to account for unification constraints, i.e. a particular item needs
  to be routed through x,y
    - -> we might have to life with this 'overconstraining' since otherwise we are
  not able to properly deal with unification in the multi-commodity min-cost
  flow

  - effect of transition location
    - when edge capacity is exhausted --> identify bottleneck, e.g., request additional support 

  
## Optimization
    - construction heuristic
        - per se a construction heuristic defines the construction of a solution according to some 'rules', but does not try to improve upon this particular solution
        - our construction heuristic is first build on the temporal/model/role/ approach
        - create an (infeasible) solution
    - local search to improve solution:
        - create a neighbourhood for a solution
        - check if the neighbourhood is a valid solution

        - conflict based refined is one option, which is used to apply only a
          minimal set of resources -- however, to actually solve the problem we
          can: add another mobile system, add immobile units -- but only with
          restrictions due to the capacities

## Simulation -- as part of solution analysis
    - take a plan and step through it --> see organization model analyser
     - allow to compute general distribution, e.g., to estimate recovery
     - allow to configure loss of individual items -- check the effect on the
       current plan: fatal, ability to recover (with the existing set of
       resources -- after replanning)
     - identify waiting times
     - show estimated values for value at time t
### Visualization
     Atomic agents (Table View)
                                   position, energy level, mass, travelled distance, wait/stand time, current activity, current time
     [Agent 0] [Composite With Role] [pos][energy][mass][distance travelled][distance to nxt goal][total wait time][transition | wait | reconfigure | functionality performance][time]
     [Agent 1] [Composite Agent] [pos][energy][mass][distance travelled][distance to nxt goal][total wait time][transition | wait | reconfigure | functionality performance][time]

     Composite agents (Tree View) -- # of agents (dormant or active)
     [Coalition: ....] --> ModelInstance: representation of a coalition including models and instance ids

### Organization Model Support
     1. OrganizationModel: (linear interpolation) input: from position - to positions, start time - end time, (on/off), activity: [join,split,active-wait,active-operative,dormant], --> output: mass, travelled distance, activity time, available functionality, metric query (at a given time t)
        - allow to compute the metric 
          - min/avg/max distance to relevant resources
          - redundancy (min/avg/max)
     2. Analyser

## Plan Execution (use FIPA message for distribution)
    - send action command to all robotic systems
    - monitor state and retrieve current status

## GUI 
 - embed pictograms as icons: QGraphicsPixmapItem
    https://stackoverflow.com/questions/5960074/qimage-in-a-qgraphics-scene
 - alternatively use color icons and allow customization

## Organization model
   - functionality constraints, e.g., manipulation only available with maximum
     of n payloads or locomotion only available with mass maximum 
   - functional property requirement
     - hasTransportCapacity max 10 Actor
     - mass max 100kg -- locomotion
     - swrl rule --> how to define in owl
     - https://www.w3.org/2007/OWL/wiki/Data_Range_Extension:_Linear_Equations +
       MathML
     - SherpaTT with mass > 100 has max 0 locomotion 
      (SherpaTT and mass > 100)
     - define for each dataProperty, which operation should be used to define to
       merge properties: sum,min,max,avg,
     - might conflict with the functional saturation bound, e.g.

    - using class expression editor of protege: Locomotion and mass some  double[<= 100]
        <owl:Class rdf:about="http://www.rock-robotics.org/2015/12/projects/TransTerrA#Locomotion100kg">
            <rdfs:subClassOf>
                <owl:Class>
                    <owl:intersectionOf rdf:parseType="Collection">
                        <rdf:Description rdf:about="&om-schema;Locomotion"/>
                        <owl:Restriction>
                            <owl:onProperty rdf:resource="&om-schema;mass"/>
                            <owl:someValuesFrom>
                                <rdfs:Datatype>
                                    <owl:onDatatype rdf:resource="&xsd;double"/>
                                    <owl:withRestrictions rdf:parseType="Collection">
                                        <rdf:Description>
                                            <xsd:maxInclusive rdf:datatype="&xsd;double">100.0</xsd:maxInclusive>
                                        </rdf:Description>
                                    </owl:withRestrictions>
                                </rdfs:Datatype>
                            </owl:someValuesFrom>
                        </owl:Restriction>
                    </owl:intersectionOf>
                </owl:Class>
            </rdfs:subClassOf>
        </owl:Class>

     

