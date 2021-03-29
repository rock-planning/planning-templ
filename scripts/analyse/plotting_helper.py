import matplotlib as mpl
import matplotlib.pyplot as plt
import os

class TTYColor:
    '''
        from https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
    '''
    Red    = '\033[91m'
    Green = '\033[92m'
    Blue = '\033[94m'
    Cyan = '\033[96m'
    White = '\033[97m'
    Yellow = '\033[93m'
    Magenta = '\033[95m'
    Grey = '\033[90m'
    Black = '\033[90m'

    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Logger:
    @staticmethod
    def warn(msg):
        print(TTYColor.Red + "Warning: " + msg + TTYColor.ENDC)

    @staticmethod
    def info(msg, color = TTYColor.Grey):
        print(color + "Info: " + msg + TTYColor.ENDC)

    @staticmethod
    def infoGreen(msg):
        Logger.info(msg, TTYColor.Green)

    @staticmethod
    def infoGrey(msg):
        Logger.info(msg, TTYColor.Grey)



class Tikz:
    @staticmethod
    def append_default_arguments(parser):
        parser.add_argument("-i","--interactive",help="use the script non-interactively",
                action="store_true")
        parser.add_argument("-f","--fontsize",help="define font size", default=12)
        parser.add_argument("-xf","--xfontsize",help="define font size for xlabels", default=12)
        parser.add_argument("-yf","--yfontsize",help="define font size for ylabels", default=12)
        parser.add_argument("-tf","--tfontsize",help="define font size for title", default=12)
        parser.add_argument("-lf","--legendfontsize",help="define font size for legend", type=int, default=None)
        parser.add_argument("-xtf","--xticksfontsize",help="define font size for xtick labels", default=12)
        parser.add_argument("-ytf","--yticksfontsize",help="define font size for ytick labels", default=12)

    @staticmethod
    def figure_width(textwidth_in_cm = 14):
        '''
        Get the figure width in inches
        '''
        return Tikz.to_inch(textwidth_in_cm)

    @staticmethod
    def figure_height(textheight_in_cm = 10):
        return Tikz.to_inch(textheight_in_cm)

    @staticmethod
    def to_inch(cm):
        return cm/2.54


    @staticmethod
    def figsize_by_ratio(width_to_height_ratio = 1.0, cm = 28, is_width = True):
        '''
            Use figsize by ratio in order to define the general figure size
            Typically used double the size in cm than the intended printed size
            for good results with matplotlib

                plt.subplots(1,1, sharex = True, figsize=tikz.Tikz.figsize_by_ratio(1.2, 10, False))
        '''
        if is_width:
            width = Tikz.figure_width(cm)
            height = width*width_to_height_ratio*1.0
        else:
            height = Tikz.figure_height(cm)
            width = height/width_to_height_ratio
        return (width, height)

    # Write the pdf (tikz) file - and show if the corresponding option is set
    #
    #
    @staticmethod
    def plot(outfilename, user_options):
        options = { 'interactive': True, 'figure': None, 'dpi': 300, 'type': 'pgf',
                'tight_layout': True,
                'align_xlabels': True,
                'crop': False }
        options.update(user_options)

        print("Tikz: plotting to {}".format(outfilename))

        directory = os.path.dirname(outfilename)
        if not os.path.isdir(directory):
            print("Create directory: {}".format(directory))
            os.mkdir(directory)

        if options['type'] == 'pgf':
            pgf_with_pdflatex = {
                "figure.autolayout": True,
                "pgf.texsystem": "pdflatex",
                "pgf.preamble": [
                     r"\usepackage[utf8x]{inputenc}",
                     r"\usepackage[T1]{fontenc}",
                     r"\usepackage{cmbright}",
                     ]
            }
            mpl.rcParams.update(pgf_with_pdflatex)

        if options['tight_layout'] == True:
            plt.tight_layout()

        # Pad margins so that markers don't get clipped by the axes
        #plt.margins(0.1)
        # Tweak spacing to prevent clipping of tick-labels
        #plt.subplots_adjust(bottom=0.05)
        if 'figure' in options.keys():
            if options['figure'] == None:
                fig = plt.gcf()
            else:
                fig = options['figure']
        else:
            fig = plt.gcf()

        #if options['align_xlabels'] == True:
            #fig.align_xlabels()

        if options['interactive'] == True:
            plt.show()

        basename = os.path.splitext(outfilename)[0]
        filetype = options['type']
        if filetype == 'png':
            basename = "{}.png".format(basename)
            fig.savefig("{}".format(basename), dpi = options['dpi'],
                    bbox_inches='tight', pad_inches=0)
        else:
            basename = "{}.{}".format(basename,filetype)
            fig.savefig("{}".format(basename))

        print("Written: {}".format(basename))
        if options['crop'] == True:
            if filetype == 'pdf':
                cmd = "pdfcrop {} {}".format(basename,basename)
                os.system(cmd)


