def dll_loader(lib, fallback_dir):
    import ctypes
    from ctypes.util import find_library
    loaded = False
    libname = find_library(lib)
    if libname:
        try:
            ctypes.CDLL(libname, ctypes.RTLD_GLOBAL)
            loaded = True
        except:
            pass
    if not loaded:
        from platform import system
        from os.path import isfile

        sys = system()
        if sys=='Windows':
            ext='.dll'
        elif sys=='Darwin':
            ext='.dylib'
        else: # Linux, other UNIX systems
            ext='.so'
        fname = fallback_dir+'/lib'+lib+ext
        if isfile(fname):
            ctypes.CDLL(fname, ctypes.RTLD_GLOBAL)
        else:
            raise ImportError

class PlanningAlgorithms(object):
    UNKNOWN = 0
    BOOL = 1
    ENUM = 2
    INT = 3
    DOUBLE = 4

    def __init__(self, module):
        self.plannerMap = {}
        for obj in dir(module):
            obj2 = '%s.%s' % (module.__name__, obj)
            self.addPlanner(obj2)

    def addPlanner(self, planner):
        from inspect import isclass
        import ompl
        if eval('isclass(%s) and issubclass(%s,ompl.base.Planner)' % (planner, planner)):
            try:
                # Get a parameter dictionary by creating a bogus planner instance.
                # Note that ompl.control.SpaceInformation is derived from
                # ompl.base.SpaceInformation and geometric planner constructors
                # happily accept an ompl.control.SpaceInformation object.
                params = eval("""%s(ompl.control.SpaceInformation(
                    ompl.base.RealVectorStateSpace(1),
                    ompl.control.RealVectorControlSpace(
                        ompl.base.RealVectorStateSpace(1),1))).params()""" % planner)
            except:
                # skip planners like Syclop that don't have a basic constructor
                return
            pnames = ompl.util.vectorString()
            params.getParamNames(pnames)
            paramMap = {}
            for pname in pnames:
                p = params[pname]
                rangeSuggestion = p.getRangeSuggestion()
                if rangeSuggestion == '': continue
                rangeSuggestion = rangeSuggestion.split(':')
                defaultValue = p.getDefaultValue()
                if len(rangeSuggestion)==1:
                    print planner
                    print rangeSuggestion
                    rangeSuggestion = rangeSuggestion[0].split(',')
                    if len(rangeSuggestion)==1:
                        raise Exception('Cannot parse range suggestion')
                    elif len(rangeSuggestion)==2:
                        rangeType = self.BOOL
                        defaultValue = False if defaultValue==rangeSuggestion[0] else True
                        rangeSuggestion = ''
                    else:
                        rangeType = self.ENUM
                else:
                    if ('.' in rangeSuggestion[0] or '.' in rangeSuggestion[-1]):
                        rangeType = self.DOUBLE
                        rangeSuggestion = [float(r) for r in rangeSuggestion]
                        defaultValue = 0. if defaultValue=='' else float(defaultValue)
                    else:
                        rangeType = self.INT
                        rangeSuggestion = [int(r) for r in rangeSuggestion]
                        defaultValue = 0 if defaultValue=='' else int(defaultValue)
                    if len(rangeSuggestion)==2:
                        rangeSuggestion = [rangeSuggestion[0], 1, rangeSuggestion[1]]
                if rangeSuggestion != '':
                    name = p.getName()
                    displayName = name.replace('_', ' ').capitalize()
                    paramMap[p.getName()] = (displayName, rangeType, rangeSuggestion, defaultValue)
            self.plannerMap[planner] = paramMap

    def getPlanners(self):
        return self.plannerMap

def initializePlannerLists():
    import ompl.geometric, ompl.control
    if ompl.geometric.planners == None:
        ompl.geometric.planners = ompl.PlanningAlgorithms(ompl.geometric)
    if ompl.control.planners == None:
        ompl.control.planners = ompl.PlanningAlgorithms(ompl.control)
