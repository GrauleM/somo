# Be sure to run this file from the "region_of_acquisition" folder
#     cd examples/region_of_acquisition
#
import yaml
import json
import time
import copy
import shutil
import os
import pybullet as p
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.tri as tri
from matplotlib import rcParams
import seaborn as sns
import sys
from natsort import natsorted


from somo.sweep import iter_utils
from somo.sweep import ContourPlotter


def dictlist_from_listdict(listdict):
    res = {}
    for key in listdict[0]:
        res[key] = []
    
    for sub in listdict: 
        for key in sub: 
            res[key].append(sub[key])
    
    return res

def default(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif np.isnan(obj):
        return None
   
    print(obj)
    raise TypeError('Not serializable')



class CondenseData:
    def __init__(self, config_file):
        self.config = iter_utils.load_yaml(config_file)

        self.status_colors_def = iter_utils.load_yaml('labels/color_sets.yaml')
        self.fingertip_color_def = self.status_colors_def.get('fingertip', [1.0, 0.7, 0.0])

        self.setup = self.config.get('setup',{})
        self.slices_2d   = self.setup.get('slices_2d',False)
        self.mirror_over_x   = False
        self.status_colors   = {'default': self.status_colors_def['default']}
        self.fingertip_color = self.fingertip_color_def
        self.emphasize_fingertip = False
        self.axes_equal = True
        self.xlabel=None
        self.ylabel=None
        self.labels_to_graph = []
        self.label_columns={}
        self.label_scales=None
        self.input_vars = None
        self.overall = {}
        self.raw = {}


    def set_axes_equal(self, in_set):
        self.axes_equal = in_set


    def set_label_columns(self,label,cols, tie_to_input, input_vals, scales=None, col_labels=None):
        if col_labels is None:
            col_labels=cols
        
        if scales == None:
            scales=[1.0]*len(cols)
        label_cols={}
        label_scales ={}
        label_labels={}
        for idx, val in enumerate(input_vals):
            label_cols[val] = cols[idx]
            label_scales[val] = scales[idx]
            label_labels[val] = col_labels[idx]

        self.label_columns[label] = {}
        self.label_columns[label]['input'] = tie_to_input
        self.label_columns[label]['scales'] = label_scales
        self.label_columns[label]['labels'] = label_labels
        self.label_columns[label]['cols'] = label_cols


    def clear_label_columns(self,label=None):
        if isinstance(label,str):
            label = [label]
        
        if isinstance(label,list):
            for label_curr in label:
                del self.label_columns[label_curr]
        
        else:
            self.label_columns = {}


    def get_data(self,filename):
        #self.results = iter_utils.load_yaml(filename)
        filename, ext = os.path.splitext(filename)
        data = iter_utils.load_yaml(filename+'_flattened'+ext)
        self.df = pd.DataFrame(data)


    def _add_data_dim(self,column, rename=None):
        if rename is None:
            rename=column

        return self.df[column].values
        


    def _calculate_condense(self,label,filter_cols=None, do_fun=None):

        self.df11 = copy.deepcopy(self.df)
        if isinstance(filter_cols,list):
            for filter_col in filter_cols:
                if isinstance(filter_col, str):
                    self.df11=self.df11[self.df11[filter_col]==1]
                elif isinstance(filter_col, dict):
                    self.df11=self.df11[self.df11[filter_col['col']]==filter_col['equals']]

        out = {}
        out['mean']=[]
        out['std']=[]
        out['percentile25']=[]
        out['percentile50']=[]
        out['percentile75']=[]

        if self.label_columns.get(label, None) is not None:
            label_settings = self.label_columns[label]
            inpt = label_settings['input']

            for input_val in label_settings['cols']:
                self.df12 = self.df11[ self.df11[inpt] == input_val ]
                Z_label=self.df12[label].dropna()

                if do_fun is not None:
                    Z_label_orig = copy.deepcopy(Z_label)
                    Z_label = []
                    for val in Z_label_orig:
                        Z_label.append(do_fun(val))
                    
                    Z_label=np.array(Z_label)

                if len(Z_label) == 0:
                    out['mean'].append(np.nan)
                    out['std'].append(np.nan)
                    out['percentile25'].append(np.nan)
                    out['percentile50'].append(np.nan)
                    out['percentile75'].append(np.nan)

                else:
                    Z_all = np.stack(Z_label)
                    z_curr=Z_all[:,label_settings['cols'][input_val]]
                    mean = np.mean(z_curr, axis=0)
                    std = np.std(z_curr, axis=0)
                    percentile25 = np.percentile(z_curr, 25, axis=0)
                    percentile50 = np.percentile(z_curr, 50, axis=0)
                    percentile75 = np.percentile(z_curr, 75, axis=0)

                    out['mean'].append(mean*label_settings['scales'][input_val])
                    out['std'].append(std*label_settings['scales'][input_val])
                    out['percentile25'].append(percentile25*label_settings['scales'][input_val])
                    out['percentile50'].append(percentile50*label_settings['scales'][input_val])
                    out['percentile75'].append(percentile75*label_settings['scales'][input_val])
        
        else:
            data =self.df11[label]

            if len(data) == 0:
                out['mean'].append(np.nan)
                out['std'].append(np.nan)
                out['percentile25'].append(np.nan)
                out['percentile50'].append(np.nan)
                out['percentile75'].append(np.nan)
            
            else:
                Z_all = np.stack(data.values)

                if do_fun is not None:
                    Z_label_orig = copy.deepcopy(Z_all)
                    Z_all = []
                    for val in Z_label_orig:
                        Z_all.append(do_fun(val))
                    
                    Z_all=np.array(Z_all)


                mean = np.mean(Z_all, axis=0)
                std = np.std(Z_all, axis=0)
                percentile25 = np.percentile(Z_all, 25, axis=0)
                percentile50 = np.percentile(Z_all, 50, axis=0)
                percentile75 = np.percentile(Z_all, 75, axis=0)
                out['mean'].append(mean)
                out['std'].append(std)
                out['percentile25'].append(percentile25)
                out['percentile50'].append(percentile50)
                out['percentile75'].append(percentile75)
            
        return out

    def _calculate_mins(self,label, min_col=None,filter_cols=None):
        self.df11 = copy.deepcopy(self.df)
        if isinstance(filter_cols,list):
            for filter_col in filter_cols:
                if isinstance(filter_col, str):
                    self.df11=self.df11[self.df11[filter_col]==1]
                elif isinstance(filter_col, dict):
                    self.df11=self.df11[self.df11[filter_col['col']]==filter_col['equals']]


        out = {}
        out['max'] = []
        out['min'] = []
        
        if self.label_columns.get(label, None) is not None:
            label_settings = self.label_columns[label]
            inpt = label_settings['input']

            for input_val in label_settings['cols']:
                self.df12 = self.df11[ self.df11[inpt] == input_val ]
                Z_label=self.df12[label].dropna()
                if len(Z_label) == 0:
                    out['max'].append(np.nan)
                    out['min'].append(np.nan)

                else:
                    max_idx = df[min_col].argmax()
                    min_idx = df[min_col].argmin()
                    Z_all = np.stack(Z_label)
                    z_curr=Z_all[:,label_settings['cols'][input_val]]


                    out['max'].append(z_curr[max_idx]*label_settings['scales'][input_val])
                    out['min'].append(z_curr[min_idx]*label_settings['scales'][input_val])
        
        else:
            data =self.df11[min_col]
            if len(data) == 0:
                out['max'].append(np.nan)
                out['min'].append(np.nan)
            else:
                max_idx = data.argmax()
                min_idx = data.argmin()
                z_curr = np.stack(self.df11[label].values).tolist()
                out['max'].append(z_curr[max_idx])
                out['min'].append(z_curr[min_idx])
            
        return out



    def _count_true(self,column, value=1.0):
        return [len(self.df[self.df[column]==value]), len(self.df)]


    def count_true(self, column, value=1.0):
        counts = self.iterate_over_function(self._count_true, column = column, value=value)
        counts = np.array(counts)

        ratio = counts[:,0]/counts[:,1]
        self.overall[column+'_count'] = counts[:,0]
        self.overall[column+'_total'] = counts[:,1] 
        self.overall[column+'_ratio'] = ratio 


    def get_data_dim(self, column, rename=None):
        return self.iterate_over_function(self._add_data_dim, column = column, rename=rename)
        

    def iterate_over_function(self, function, **kwargs):
        base_folder = iter_utils.get_group_folder(self.config)
        print(base_folder)

        if self.slices_2d:
            dir_list = os.listdir(base_folder)
            dir_list = natsorted(dir_list)
            folders = [os.path.join(base_folder, subdir) for subdir in dir_list if os.path.isdir(os.path.join(base_folder, subdir))]
        else:
            folders=['']
        
        out = []
        for folder in folders:
            print(folder)
            success_filename = os.path.join(base_folder,folder,'summary.yaml')
            self.get_data(success_filename)
            fun_out = function(**kwargs)
            out.append(fun_out)
        
        return out

    def get_vars(self):
        base_folder = iter_utils.get_group_folder(self.config)
        settings = iter_utils.load_yaml(os.path.join(base_folder, 'summary.yaml'))
        variables = settings['vars']
        sweep = settings['sweep']

        for idx, var in enumerate(variables):
            self.overall[var] = sweep[idx]



    def plot(self, x, y, labels, suffix='', **kwargs):

        curr_data=copy.deepcopy(self.overall)

        if isinstance(self.overall[y],dict):
            for key in curr_data[y]:
                curr_data[y+'_'+key] = curr_data[y][key]
            del curr_data[y]

        for key in self.overall:
            if not(key == x or key==y):
                del curr_data[key]


        df = pd.DataFrame(curr_data)
        df = df.sort_values([x])

        if isinstance(self.overall[y],dict):
            y_vals = np.stack(df[y+'_mean'].values).T.tolist()
            y_err = np.stack(df[y+'_std'].values).T.tolist()
            
            for y_entry, y_error, label in zip(y_vals,y_err,labels):
                y_entry=np.array(y_entry)
                y_error=np.array(y_error)
                plt.plot(df[x].values, y_entry, label=label, **kwargs)
                plt.fill_between(df[x].values, y_entry-y_error, y_entry+y_error,alpha=0.3)
        else:
            y_vals_np = np.stack(df[y].values)
            y_vals = y_vals_np.T.tolist()

            if min(y_vals_np.shape)==1 or y_vals_np.ndim==1:
                plt.plot(df[x].values, y_vals, label=labels[0], **kwargs)
            else:
                for y_entry, label in zip(y_vals,labels):
                    plt.plot(df[x].values, y_entry, label=label, **kwargs)

        plt.xlabel(x)
        plt.ylabel(y)
        plt.legend()

        base_folder = iter_utils.get_group_folder(self.config)
        figname = os.path.join(base_folder,str(y) +'_vs_'+str(x)+suffix+'.png')
        plt.savefig(figname, dpi=450)
        plt.savefig(figname.replace('.png','.svg'), dpi=450)

        plt.close()






    def contour_plot(self, x, y, z, labels, suffix='', subfield=None ,**kwargs):

        curr_data=copy.deepcopy(self.overall)

        if isinstance(self.overall[z],dict):
            for key in curr_data[z]:
                curr_data[z+'_'+key] = curr_data[z][key]
            del curr_data[z]

        for key in self.overall:
            if not(key == x or key==y or key==z):
                del curr_data[key]

        df = pd.DataFrame(curr_data)
        df = df.sort_values([x, y])

        plt.close()
        fig = plt.figure(**kwargs.get('figure',{}))
        fig.patch.set_facecolor('w')
        if isinstance(self.overall[z],dict):
            z_vals = np.stack(df[z+'_'+subfield].values).T.tolist()
            #y_err = np.stack(df[z+'_std'].values).T.tolist()
            length = len(labels)
            for z_values, label, idx in zip(z_vals,labels, range(len(labels))):
                mask=np.isnan(z_values)
                z_entry=np.array(z_values)[~mask]
                x_entry=df[x].values[~mask]
                y_entry=df[y].values[~mask]
                
                #diff = (max(z_values) - min(z_values))
                #marker_vals = np.array(z_values)/max(z_values)*60
                #print(marker_vals)
                

                plt.subplot(1,length, idx+1)

                if kwargs['contour'].get('levels', None) is None:
                    num_levels=kwargs['contour'].get('contour_levels',10)
                    levels=np.linspace(min(z_entry)-0.000001, max(z_entry)+0.000001,num_levels)
                    plt.tricontourf(x_entry, y_entry, z_entry, levels=levels, **kwargs['contour'])
                
                else:
                    plt.tricontourf(x_entry, y_entry, z_entry, **kwargs['contour'])
                plot = plt.plot(df[x].values, df[y].values, **kwargs['points'])
                plot[0].set_clip_on(False)

                plt.title(label)

                if idx ==0:
                    plt.ylabel(y)
                
                if idx == np.ceil(length/2)-1 or length==1:
                    plt.xlabel(x)
                    
        else:
            z_vals_np = np.stack(df[z].values)
            z_vals = z_vals_np.T.tolist()

            if min(z_vals_np.shape)==1 or z_vals_np.ndim==1:
                if kwargs['contour'].get('levels', None) is None:
                    num_levels=kwargs['contour'].get('contour_levels',10)
                    levels=np.linspace(min(z_vals_np)-0.000001, max(z_vals_np)+0.000001,num_levels)
                    print(levels)
                    plt.tricontourf(df[x].values, df[y].values, z_vals, levels=levels, **kwargs['contour'])
                
                plt.tricontourf(df[x].values, df[y].values, z_vals, **kwargs['contour'])

                plt.plot(df[x].values, df[y].values, **kwargs['points'])
                plt.title(labels)
            else:
                length = len(labels)
                for z_values, label, idx in zip(z_vals,labels, range(len(labels))):
                    mask=np.isnan(z_values)
                    z_entry=np.array(z_values)[~mask]
                    x_entry=df[x].values[~mask]
                    y_entry=df[y].values[~mask]

                    #diff = (max(z_values) - min(z_values))
                    #marker_vals = np.array(z_values)/max(z_values)*60
                    #print(marker_vals)

                    plt.subplot(1,length, idx+1)

                    if kwargs['contour'].get('levels', None) is None:
                        num_levels=kwargs['contour'].get('contour_levels',10)
                        levels=np.linspace(min(z_entry)-0.000001, max(z_entry)+0.000001,num_levels)
                        plt.tricontourf(x_entry, y_entry, z_entry, levels=levels, **kwargs['contour'])

                    else:
                        plt.tricontourf(x_entry, y_entry, z_entry, **kwargs['contour'])
                    plot = plt.plot(df[x].values, df[y].values, **kwargs['points'])

                    plot[0].set_clip_on(False)
                    plt.title(label)
                    

                    if idx ==0:
                        plt.ylabel(y)
                    
                    if idx == np.ceil(length/2)-1 or length==1:
                        plt.xlabel(x)

        
        cbar = plt.colorbar()
        cbar.set_label(z)

        base_folder = iter_utils.get_group_folder(self.config)
        figname = os.path.join(base_folder,str(z) +'_vs_'+str(x)+'_and_'+str(y)+suffix+'_contour.png')
        plt.savefig(figname, dpi=450)
        plt.savefig(figname.replace('.png','.svg'), dpi=450)

        return figname



    def plot_percentiles(self, x, y, labels, suffix='', **kwargs):

        curr_data=copy.deepcopy(self.overall)

        if isinstance(self.overall[y],dict):
            for key in curr_data[y]:
                curr_data[y+'_'+key] = curr_data[y][key]
            del curr_data[y]

        for key in self.overall:
            if not(key == x or key==y):
                del curr_data[key]


        df = pd.DataFrame(curr_data)
        df = df.sort_values([x])

        if isinstance(self.overall[y],dict):
            y_vals = np.stack(df[y+'_percentile50'].values).T.tolist()
            y_err_pos = np.stack(df[y+'_percentile75'].values).T.tolist()
            y_err_neg = np.stack(df[y+'_percentile25'].values).T.tolist()
            
            for y_entry, y_error_p, y_error_n , label in zip(y_vals,y_err_pos, y_err_neg,labels):
                y_entry=np.array(y_entry)
                y_error_p=np.array(y_error_p)
                y_error_n=np.array(y_error_n)
                plt.plot(df[x].values, y_entry, label=label, **kwargs)
                plt.fill_between(df[x].values, y_entry-y_error_n, y_entry+y_error_p,alpha=0.3)
        else:
            y_vals = np.stack(df[y].values).T.tolist()

            for y_entry, label in zip(y_vals,labels):
                plt.plot(df[x].values, y_entry, label=label, **kwargs)

        plt.xlabel(x)
        plt.ylabel(y)
        plt.legend()

        base_folder = iter_utils.get_group_folder(self.config)
        figname = os.path.join(base_folder,str(y) +'_vs_'+str(x)+suffix+'.png')
        plt.savefig(figname, dpi=450)
        plt.savefig(figname.replace('.png','.svg'), dpi=450)

        plt.close()

    
    def condense(self, labels=None, inputs=None, filter_cols=None, **kwargs):

        for label in labels:
            dat = self.iterate_over_function(self._calculate_condense, label = label, filter_cols=filter_cols, **kwargs)
            dat_list = dictlist_from_listdict(dat)
            self.overall[label] = dat_list

    def find_mins(self, labels=None, inputs=None, min_col=None, filter_cols=None):

        for label in labels:
            dat = self.iterate_over_function(self._calculate_mins, label = label, min_col=min_col, filter_cols=filter_cols)
            dat_list = dictlist_from_listdict(dat)
            self.overall[label+'_min_'+min_col] = dat_list

    
    def save_summary(self, filename=None):
        if filename is None:
            folder=iter_utils.get_group_folder(self.config)
            filename=os.path.join(folder,'label_summary.json')
        
        with open(filename,'w') as f:
            json.dump(self.overall, f, default=default)

    def load_summary(self, filename=None):
        if filename is None:
            folder=iter_utils.get_group_folder(self.config)
            filename=os.path.join(folder,'label_summary.json')
        
        with open(filename,'r') as f:
            self.overall = json.load(f)

