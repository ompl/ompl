######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2016, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

library(pool)
library(dplyr, warn.conflicts = FALSE)
library(tidyr)
library(ggplot2)
library(rlang)
library(Hmisc)

default_database <- "www/benchmark.db"

no_database_text <-
    "No database loaded yet. Upload one by clicking on “Change database”."
not_ready_text <-
    "The benchmarking results are not available yet, check back later."
sessions_folder <- "/tmp/omplweb_sessions"
problem_params_aggregate_text <- "all (aggregate)"
problem_params_separate_text <- "all (separate)"

problem_select_widget <- function(problems, widget_name) {
    widget <- selectInput(widget_name,
        label = h4("Motion planning problem"),
        choices = problems)
    conditional_disable(widget, length(problems) < 2)
}

problem_param_select <- function(param, val) {
    if (val == problem_params_aggregate_text ||
        val == problem_params_separate_text)
        # select all
        TRUE
    else {
        # select specific parameter value.
        # Use fuzzy matching when comparing numbers because precision is lost
        # when real-valued parameter values are converted to strings for
        # parameter selection widget.
        p <- rlang::sym(param)
        if (is.numeric(val))
        {
            v <- as.numeric(val)
            expr(abs(!!p - !!v) < 0.0000001)
        }
        else
            expr(!!p == !!val)
    }
}

# return parameters of parametrized benchmarks if they exist, NULL otherwise
problem_params <- function(experiments) {
    params <- tbl_vars(experiments)
    num_params <- length(params)
    if (num_params > 12)
        param_names <- params[13:num_params]
    else
        param_names <- NULL
}
# return selected values of benchmark parameters
problem_param_values <- function(experiments, prefix, input) {
    sapply(problem_params(experiments),
        function(name) input[[paste0(prefix, "problem_param", name)]])
}
# determine whether a performance attribute should be grouped by a benchmark
# parameter value
problem_param_group_by <- function(values) {
    grouping <- match(problem_params_separate_text, values)
    if (is.na(grouping))
        NULL
    else
        names(values)[grouping]
}
# create a widget for a given benchmark parameter
problem_param_select_widget <- function(name, experiments, prefix, problem,
                                        version) {
    values <- (experiments %>%
        filter(experiment == !!problem & version == !!version) %>%
        select(!!name) %>% distinct() %>% drop_na())[[name]]
    disp_name <- gsub("_", " ", name)
    internal_name <- paste0(prefix, "problem_param", name)
    if (length(values) == 1) {
        # don't show any widget for parameter if the only value is NA
        # (this means that the given benchmark parameter is not applicable to
        # the currently selected benchmark problem)
        if (!is.na(values[1]))
            # disable selection if there is only value for parameter
            disable(selectInput(internal_name, label = h6(disp_name),
                choices = values))
    }
    else
        selectInput(internal_name,
            label = h6(disp_name),
            choices = append(values,
                c(problem_params_aggregate_text, problem_params_separate_text),
                0))
}
# create widgets for all benchmark parameters
problem_param_select_widgets <- function(experiments, prefix, problem,
                                         version) {
    params <- problem_params(experiments)
    if (!is.null(params))
        div(class = "well well-light",
            h5("Problem parameters"),
            lapply(params, problem_param_select_widget,
                experiments = experiments, prefix = prefix, problem = problem,
                version = version)
        )
}

version_select_widget <- function(experiments, widget_name, checkbox) {
    versions <- (experiments %>% select(version) %>% distinct())$version
    if (checkbox)
        widget <- checkboxGroupInput(
            widget_name, label = h4("Selected versions"),
            choices = versions,
            selected = versions)
    else
        widget <- selectInput(widget_name, label = h4("Version"),
            choices = versions,
            # select most recent version by default
            selected = tail(versions, n = 1))
    conditional_disable(widget, length(versions) < 2)
}

planner_name_mapping <- function(fullname) {
    sub("control_", " ", sub("geometric_", "", fullname))
}
planner_select_widget <- function(performance, widget_name) {
    planners <- (performance %>% select(planner) %>% distinct())$planner
    names(planners) <- planner_name_mapping(planners)
    # select first 4 planners (or all if there are less than 4)
    if (length(planners) < 4)
        selection <- planners
    else
        selection <- planners[1:4]
    #conditionalDisable(
        checkboxGroupInput(widget_name, label = h4("Selected planners"),
        choices = planners, selected = selection)#, length(planners) < 2)
}

perf_attr_select_widget <- function(runs, widget_name) {
    attrs <- tbl_vars(runs)
    names(attrs) <- gsub("_", " ", gsub("^run\\.", "", attrs))
    if ("time" %in% attrs)
        selection <- "time"
    else
        selection <- NULL
    # strip off first 3 names, which correspond to internal id's
    selectInput(widget_name, label = h4("Benchmark attribute"),
        choices = attrs[4:length(attrs)], selected = selection)
}

shinyServer(function(input, output, session) {
    # Create a connection to a database. There are three possibilities:
    # 1. The user is using the default database.
    # 2. The user has upload their own database.
    # 3. The user has submitted a benchmark job via the OMPL.app web app and
    #    the user wants to look at the database generated by this job.
    con <- reactive({
        query <- parseQueryString(session$clientData$url_search)

        if (is.null(query$user) || is.null(query$job)) {
            if (is.null(input$database) || is.null(input$database$datapath))
                # case 1
                database <- default_database
            else
                # case 2
                database <- input$database$datapath
        } else
            # case 3
            database <- paste(sessions_folder, query$user, query$job, sep = "/")
        if (file.exists(database)) {
            con <- dbPool(RSQLite::SQLite(), dbname = database)
            # benchmark job may not yet be finished so check that "experiments"
            # table exists.
            if ("experiments" %in% DBI::dbListTables(con)) {
                updateTabsetPanel(session, "navbar", selected = "performance")
                con
            } else {
                js$refresh()
                NULL
            }
        } else
            NULL
    })

    # create variables for tables in database
    experiments <- reactive({
        con() %>% tbl("experiments") %>%
            rename(experiment.id = id, experiment = name) %>% collect()
    })
    planner_configs <- reactive({
        con() %>% tbl("plannerConfigs") %>%
            rename(planner.id = id, planner = name) %>% collect()
    })
    runs <- reactive({
        con() %>% tbl("runs") %>% rename(run.id = id) %>% collect()
    })
    attrs_names <- reactive({
        rnames <- tbl_vars(runs())
        names(rnames) <- gsub("_", " ", rnames)
        rnames
    })
    progress <- reactive({
        con() %>% tbl("progress") %>% collect()
    })
    prog_attrs_names <- reactive({
        rnames <- tbl_vars(progress())
        names(rnames) <- gsub("_", " ", gsub("\\.progress$", "", rnames))
        rnames
    })
    enums <- reactive({
        con() %>% tbl("enums") %>% collect()
    })
    runs_ext <- reactive({
        planner_configs() %>%
        inner_join(runs(), c("planner.id" = "plannerid"),
                   suffix = c(".planner", ".run")) %>%
        inner_join(experiments(),
                   c("experimentid" = "experiment.id"),
                   suffix = c(".plannerrun", ".experiment"))
    })
    performance <- reactive({
        runs_ext() %>% filter(experiment == !!input$perf_problem &
                              version == !!input$perf_version)
    })
    prog_perf <- reactive({
        runs_ext() %>%
        inner_join(progress(), c("run.id" = "runid"),
                   suffix = c("", ".progress")) %>%
        filter(experiment == !!input$prog_problem & version == !!input$prog_version)
    })
    regr_perf <- reactive({
        runs_ext() %>% filter(experiment == !!input$regr_problem &
                              version %in% !!input$regr_versions)
    })

    # Go straight to the database upload page if there is no default database
    observe({
        if (is.null(con()))
            updateTabsetPanel(session, "navbar", selected = "database")
    })

    problem_names <- reactive({
        (experiments() %>% select(experiment) %>% distinct())$experiment
    })
    output$perf_problem_select <- renderUI({
        problem_select_widget(problem_names(), "perf_problem")
    })
    output$prog_problem_select <- renderUI({
        problem_select_widget(problem_names(), "prog_problem")
    })
    output$regr_problem_select <- renderUI({
        problem_select_widget(problem_names(), "regr_problem")
    })

    output$perf_problem_param_select <- renderUI({
        validate(
            need(input$perf_problem, "Select a problem"),
            need(input$perf_version, "Select a version")
        )
        problem_param_select_widgets(experiments(), "perf", input$perf_problem,
                                     input$perf_version)
    })
    output$prog_problem_param_select <- renderUI({
        validate(
            need(input$prog_problem, "Select a problem"),
            need(input$prog_version, "Select a version")
        )
        problem_param_select_widgets(experiments(), "prog", input$prog_problem,
                                     input$prog_version)
    })
    output$regr_problem_param_select <- renderUI({
        validate(
            need(input$regr_problem, "Select a problem"),
            need(input$regr_versions, "Select a version")
        )
        problem_param_select_widgets(experiments(), "regr", input$regr_problem,
                                     tail(input$regr_versions, n = 1))
    })

    output$perf_attr_select <- renderUI({
        tagList(
            perf_attr_select_widget(runs(), "perf_attr"),
            checkboxInput("perf_show_adv_options", "Show advanced options",
                          FALSE),
            conditionalPanel(condition = "input.perf_show_adv_options",
                div(class = "well well-light",
                    checkboxInput("perf_show_as_cdf",
                        label = "Show as cumulative distribution function"),
                    checkboxInput("perf_show_simplified",
                        label = "Include results after simplification"),
                    checkboxInput("perf_show_param_box_plots",
                        label = "Show box plots for parametrized benchmarks"),
                    checkboxInput("perf_hide_outliers",
                        label = "Hide outliers in box plots"),
                    checkboxInput("perf_y_log_scale",
                        label = "Use log scale for Y-axis")
                )
            )
        )
    })
    output$regr_attr_select <- renderUI({
        perf_attr_select_widget(runs(), "regr_attr")
    })
    output$prog_attr_select <- renderUI({
        progress_attrs <- tbl_vars(progress())
        names(progress_attrs) <- gsub("_", " ", progress_attrs)
        # strip off first 2 names, which correspond to an internal id and time
        attrs <- progress_attrs[3:length(progress_attrs)]
        tagList(
            conditional_disable(selectInput("progress",
                label = h4("Progress attribute"),
                choices = attrs
            ), length(attrs) < 2),
            checkboxInput("prog_show_adv_options", "Show advanced options",
                          FALSE),
            conditionalPanel(condition = "input.prog_show_adv_options",
                div(class = "well well-light",
                    checkboxInput("progress_show_measurements",
                        label = "Show individual measurements"),
                    sliderInput("progress_opacity",
                        label = "Measurement opacity", 0, 100, 50)
                )
            )
        )
    })

    output$perf_version_select <- renderUI({
        version_select_widget(experiments(), "perf_version", FALSE)
    })
    output$prog_version_select <- renderUI({
        version_select_widget(experiments(), "prog_version", FALSE)
    })
    output$regr_version_select <- renderUI({
        version_select_widget(experiments(), "regr_versions", TRUE)
    })

    output$perf_planner_select <- renderUI({
        validate(
            need(input$perf_problem, "Select a problem"),
            need(input$perf_version, "Select a version")
        )
        planner_select_widget(performance(), "perf_planners")
    })
    output$prog_planner_select <- renderUI({
        validate(
            need(input$prog_problem, "Select a problem"),
            need(input$prog_version, "Select a version")
        )
        planner_select_widget(prog_perf(), "prog_planners")
    })
    output$regr_planner_select <- renderUI({
        validate(
            need(input$regr_problem, "Select a problem"),
            need(input$regr_versions, "Select a version")
        )
        planner_select_widget(regr_perf(), "regr_planners")
    })


    output$benchmark_info <- renderTable({
        validate(need(experiments(), no_database_text))
        validate(need(input$perf_version,
            "Select a version on the “Overall performance” page"))
        experiments() %>%
            filter(experiment == !!input$perf_problem &
                   version == !!input$perf_version) %>% t()
    }, rownames = TRUE, colnames = FALSE)
    output$planner_configs <- renderTable({
        performance() %>%
            select(planner, settings) %>%
            distinct()
    }, rownames = FALSE, colnames = FALSE)

    # font selection
    font_selection <- reactive({
        element_text(family = input$font_family, size = input$font_size)
    })

    # plot of overall performance
    perf_plot <- reactive({
        # performance results for parametrized benchmarks can be grouped by
        # parameter values
        param_values <- problem_param_values(experiments(), "perf", input)
        grouping <- problem_param_group_by(param_values)
        # for certain performance metrics we can also include the results
        # after path simplification
        attribs <- tbl_vars(runs())
        attr <- input$perf_attr
        disp_attr <- names(attrs_names()[attrs_names() == attr])
        simplified_attr <- gsub("^", "simplified_", attr)
        include_simplified_attr <- input$perf_show_simplified &&
                                 simplified_attr %in% attribs
        # compute the selection of columns and their new names
        selection <- c(planner = "planner", attr = attr)
        if (include_simplified_attr)
            selection <- c(selection, simplified_attr = simplified_attr)
        if (!is.null(grouping))
            selection <- c(selection, grouping = grouping)
        # compute selection of rows (add empty string to work around bug if
        # there is only one planner selected)
        selected_planners <- c(input$perf_planners, "")
        filter_expr <- expr(planner %in% !!selected_planners)
        # for parametrized benchmarks we want only the data that matches all
        # parameters exactly
        if (length(param_values) > 0) {
            filter_expr <- mapply(
                    problem_param_select, names(param_values), param_values, USE.NAMES = FALSE)
        }
        else
            filter_expr <- TRUE
        # extract the data to be plotted
        data <- performance() %>%
            filter(planner %in% !!selected_planners, !!!filter_expr) %>%
            select(selection)
        # turn the planner and grouping columns into factors ordered in the
        # same way that they occur in the database.
        uplanner <- unique(data$planner)
        data$planner <- factor(data$planner, uplanner,
            labels = sapply(uplanner, planner_name_mapping))
        if (!is.null(grouping))
            data$grouping <- factor(data$grouping)
        # use bar charts for enum types
        enum <- enums() %>% filter(name == attr)
        if (nrow(enum) > 0) {
            val <- enum$value
            names(val) <- enum$description
            attr_as_factor <- factor(data$attr, levels = val)
            levels(attr_as_factor) <- enum$description
            p <- qplot(planner, data = data, geom = "bar",
                      fill = attr_as_factor) +
                # labels
                theme(legend.title = element_blank(), text = font_selection())
            if (!is.null(grouping))
                p <- p + facet_grid(. ~ grouping)
        }
        else {
            if (input$perf_hide_outliers)
                outlier.shape <- NA
            else
                outlier.shape <- 16
            if (include_simplified_attr) {
                # the "all (separate)" case is not handled here
                data <- data %>%
                    gather(key, value, c(attr, simplified_attr),
                           factor_key = TRUE)
                if (input$perf_show_as_cdf)
                    p <- ggplot(data, aes(x = value, color = planner,
                        group = interaction(planner, key), linetype = key)) +
                        # labels
                        xlab(input$perf_attr) +
                        ylab("cumulative probability") +
                        theme(text = font_selection()) +
                        # empirical cumulative distribution function
                        stat_ecdf(size = 1) +
                        scale_linetype_discrete(name = "",
                            labels = c("before simplification",
                                "after simplification"))
                else {
                    p <- ggplot(data, aes(x = planner, y = value, color = key,
                                fill = key)) +
                        # labels
                        ylab(disp_attr) +
                        theme(legend.title = element_blank(),
                              text = font_selection()) +
                        geom_boxplot(outlier.shape = outlier.shape,
                                     na.rm = TRUE) +
                        scale_fill_manual(values = c("#99c9eb", "#ebc999"),
                            labels = c("before simplification",
                                       "after simplification")) +
                        scale_color_manual(values = c("#3073ba", "#ba7330"),
                            labels = c("before simplification",
                                       "after simplification"))
                    if (input$perf_hide_outliers)
                        lims <- quantile(data$value, c(0.2, 0.8), TRUE)
                    else
                        lims <- c(NA, NA)
                    if (input$perf_y_log_scale)
                        p <- p + scale_y_log10(limits = lims)
                    else
                        p <- p + scale_y_continuous(limits = lims)
                }
            }
            else {
                if (input$perf_show_as_cdf) {
                    if (is.null(grouping))
                        p <- ggplot(data, aes(x = attr, color = planner))
                    else
                        p <- ggplot(data, aes(x = attr, color = planner,
                                    linetype = grouping)) +
                            scale_linetype(name = gsub(" ", "_", grouping))
                    p <- p +
                        # labels
                        xlab(input$perf_attr) +
                        ylab("cumulative probability") +
                        theme(text = font_selection()) +
                        # empirical cumulative distribution function
                        stat_ecdf(size = 1)
                } else {
                    if (input$perf_show_param_box_plots ||
                        is.null(grouping)) {
                        p <- ggplot(data, aes(x = planner, y = attr,
                                    group = planner)) +
                            # labels
                            ylab(disp_attr) +
                            theme(legend.position = "none",
                                  text = font_selection()) +
                            # box plots for boolean, integer, and real-valued
                            # attributes
                            geom_boxplot(color = I("#3073ba"),
                                fill = I("#99c9eb"),
                                outlier.shape = outlier.shape,
                                na.rm = TRUE)
                        if (!is.null(grouping))
                            p <- p + facet_grid(. ~ grouping)
                        if (input$perf_hide_outliers)
                            lims <- quantile(data$attr, c(0.2, 0.8), TRUE)
                        else
                            lims <- c(NA, NA)
                    } else {
                        p <- ggplot(data, aes(x = planner, y = attr,
                                    color = grouping, fill = grouping)) +
                            # labels
                            ylab(disp_attr) +
                            theme(legend.title = element_blank(),
                                  text = font_selection()) +
                            geom_boxplot(position = "dodge",
                                outlier.shape = outlier.shape, na.rm = TRUE) +
                            scale_x_discrete(expand = c(0.05, 0))
                            if (input$perf_hide_outliers)
                                lims <- quantile(data$attr, c(0.2, 0.8), TRUE)
                            else
                                lims <- c(NA, NA)
                    }
                    if (input$perf_y_log_scale)
                        p <- p + scale_y_log10(limits = lims)
                    else
                        p <- p + scale_y_continuous(limits = lims)
                }
            }
        }
        p
    })
    output$perf_plot <- renderPlot({
        validate(
            need(input$perf_version, "Select a version"),
            need(input$perf_problem, "Select a problem"),
            need(input$perf_attr, "Select a benchmark attribute"),
            need(input$perf_planners, "Select some planners")
        )
        print(perf_plot())
    })
    output$perf_download_plot <- downloadHandler(filename = "perfplot.pdf",
        content = function(file) {
            pdf(file = file, width = input$paper_width,
                height = input$paper_height)
            print(perf_plot())
            dev.off()
        }
    )
    output$perf_download_rdata <- downloadHandler(filename = "perfplot.rds",
        content = function(file) {
            saveRDS(perf_plot(), file = file)
        }
    )
    output$perf_missing_data_table <- renderTable({
        validate(
            need(input$perf_version, "Select a version"),
            need(input$perf_problem, "Select a problem"),
            need(input$perf_attr, "Select a benchmark attribute"),
            need(input$perf_planners, "Select some planners")
        )
        attr <- input$perf_attr
        # performance results for parametrized benchmarks can be grouped by
        # parameter values
        param_values <- problem_param_values(experiments(), "perf", input)
        grouping <- problem_param_group_by(param_values)
        selected_planners <- c(input$perf_planners, "")
        filter_expr <- expr(planner %in% !!selected_planners)
        data <- performance() %>%
            filter(!!filter_expr) %>%
            group_by(planner)
        if (!is.null(grouping))
            # TODO: avoid using deprecated group_by_ function
            data <- data %>% group_by(!!rlang::sym(grouping), add = TRUE)
        data <- data %>%
            select(attr = !!attr) %>%
            mutate(missing = is.na(attr)) %>%
            summarise(missing = sum(missing, na.rm = TRUE), total = n())
        data$planner <- factor(data$planner, unique(data$planner),
            labels = sapply(unique(data$planner), planner_name_mapping))
        data
    }, rownames = FALSE)

    # progress plot
    prog_plot_data <- reactive({
        validate(
            need(input$prog_version, "Select a version"),
            need(input$prog_problem, "Select a problem"),
            need(input$progress, "Select a benchmark attribute"),
            need(input$prog_planners, "Select some planners")
        )
        # performance results for parametrized benchmarks can be grouped by
        # parameter values
        param_values <- problem_param_values(experiments(), "prog", input)
        grouping <- problem_param_group_by(param_values)
        attr <- gsub(" ", "_", input$progress)
        selection <- c(planner = "planner", time = "time.progress", attr = attr)
        if (!is.null(grouping))
            selection <- c(selection, grouping = grouping)
        # compute selection of rows (add empty string to work around bug if
        # there is only one planner selected)
        selected_planners <- c(input$prog_planners, "")
        if (length(param_values) > 0)
            filter_expr <- mapply(
                problem_param_select, names(param_values), param_values, USE.NAMES = FALSE)
        else
            filter_expr <- TRUE
        data <- prog_perf() %>%
            filter(planner %in% !!selected_planners & !is.na(!!attr), !!!filter_expr) %>%
            select(selection)
        # turn the planner and grouping columns into factors ordered in the
        # same way that they occur in the database.
        data$planner <- factor(data$planner, unique(data$planner),
            labels = sapply(unique(data$planner), planner_name_mapping))
        if (!is.null(grouping))
            data$grouping <- factor(data$grouping)
        list(data = data, grouping = grouping)
    })
    prog_plot <- reactive({
        attr <- gsub(" ", "_", input$progress)
        disp_attr <- names(prog_attrs_names()[prog_attrs_names() == attr])
        attr <- paste(attr, ".progress")
        progdata <- prog_plot_data()
        data <- progdata$data
        grouping <- progdata$grouping
        validate(need(nrow(data) > 0,
            "No progress data available; select a different benchmark, progress attribute, or planners."))
        p <- ggplot(data, aes(x = time, y = attr, group = planner,
                    color = planner, fill = planner)) +
            # labels
            xlab("time (s)") +
            ylab(disp_attr) +
            theme(text = font_selection()) +
            # smooth interpolating curve
            geom_smooth(method = "gam") +
            coord_cartesian(xlim = c(0, trunc(max(data$time))))
        # optionally, add individual measurements as semi-transparent points
        if (input$progress_show_measurements)
            p <- p + geom_point(alpha = I(input$progress_opacity / 100))
        if (!is.null(grouping))
            p <- p + facet_grid(grouping ~ .)
        p
    })
    output$prog_plot <- renderPlot({
        prog_plot()
    })
    prog_num_measurements_plot <- reactive({
        attr <- gsub(" ", "_", input$progress)
        disp_attr <- names(prog_attrs_names()[prog_attrs_names() == attr])
        progdata <- prog_plot_data()
        data <- progdata$data
        grouping <- progdata$grouping
        if (nrow(data) > 0) {
            p <- ggplot(data, aes(x = time, group = planner, color = planner)) +
                # labels
                xlab("time (s)") +
                ylab(sprintf("# measurements for %s", disp_attr)) +
                theme(text = font_selection()) +
                geom_freqpoly(binwidth = 1) +
                coord_cartesian(xlim = c(0, trunc(max(data$time))))
            if (!is.null(grouping))
                p <- p + facet_grid(grouping ~ .)
            p
        }
    })
    output$prog_num_measurements_plot <- renderPlot({
        prog_num_measurements_plot()
    })
    output$prog_download_plot <- downloadHandler(filename = "progplot.pdf",
        content = function(file) {
            pdf(file = file, width = input$paper_width,
                height = input$paper_height)
            print(prog_plot())
            print(prog_num_measurements_plot())
            dev.off()
        }
    )
    output$prog_download_rdata <- downloadHandler(filename = "progplot.RData",
        content = function(file) {
            progplot <- prog_plot()
            prognummeasurementsplot <- prog_num_measurements_plot()
            save(progplot, prognummeasurementsplot, file = file)
        }
    )

    # regression plot
    regr_plot <- reactive({
        param_values <- problem_param_values(experiments(), "regr", input)
        grouping <- problem_param_group_by(param_values)
        attr <- input$regr_attr
        disp_attr <- names(attrs_names()[attrs_names() == attr])
        # compute the selection of columns and their new names
        selection <- c(planner = "planner", attr = attr, version = "version")
        if (!is.null(grouping))
            selection <- c(selection, grouping = grouping)
        # compute selection of rows (add empty string to work around bug if
        # there is only one planner selected)
        selected_planners <- c(input$regr_planners, "")
        selected_versions <- c(input$regr_versions, "")
        # for parametrized benchmarks we want only the data that matches all
        # parameters exactly
        if (length(param_values) > 0)
            filter_expr <- mapply(
                problem_param_select, names(param_values), param_values, USE.NAMES = FALSE)
        else
            filter_expr <- TRUE
        data <- regr_perf() %>%
            filter(planner %in% !!selected_planners &
                   version %in% !!selected_versions,
                   !!!filter_expr) %>%
            select(selection)

        # turn the planner and grouping columns into factors ordered in the
        # same way that they occur in the database.
        data$planner <- factor(data$planner, unique(data$planner),
            labels = sapply(unique(data$planner), planner_name_mapping))
        if (!is.null(grouping))
            data$grouping <- factor(data$grouping)
        # strip "OMPL " prefix, so we can fit more labels on the X-axis
        data$version <- sapply(data$version,
            function(str) {
                # assume the version number is the last "word" in the string
                tail(strsplit(str, " ")[[1]], n = 1)
            })
        # order by order listed in data frame (i.e., "0.9.*" before "0.10.*")
        data$version <- factor(data$version, unique(data$version))
        p <- ggplot(data, aes(x = version, y = attr, fill = planner,
                              group = planner)) +
            # labels
            ylab(disp_attr) +
            theme(legend.title = element_blank(), text = font_selection()) +
            # plot mean and error bars
            stat_summary(fun.data = "mean_cl_boot", geom = "bar",
                         position = position_dodge()) +
            stat_summary(fun.data = "mean_cl_boot", geom = "errorbar",
                         position = position_dodge())
        if (!is.null(grouping))
            p <- p + facet_grid(grouping ~ .)
        p
    })
    output$regr_plot <- renderPlot({
        validate(
            need(input$regr_versions, "Select a version"),
            need(input$regr_problem, "Select a problem"),
            need(input$regr_attr, "Select a benchmark attribute"),
            need(input$regr_planners, "Select some planners")
        )
        print(regr_plot())
    })
    output$regr_download_plot <- downloadHandler(filename = "regrplot.pdf",
        content = function(file) {
            pdf(file = file, width = input$paper_width,
                height = input$paper_height)
            print(regr_plot())
            dev.off()
        }
    )
    output$regr_download_rdata <- downloadHandler(filename = "regrplot.rds",
        content = function(file) {
            regrplot <- regr_plot()
            saveRDS(regrplot, file = file)
        }
    )

    output$performance_page <- renderUI({
        sidebarLayout(
            sidebarPanel(
                uiOutput("perf_problem_select"),
                uiOutput("perf_problem_param_select"),
                uiOutput("perf_attr_select"),
                uiOutput("perf_version_select"),
                uiOutput("perf_planner_select")
            ),
            mainPanel(
                HTML(
                    "<div class=\"alert alert-info alert-dismissible fade in\" role=\"alert\">
                      <button type=\"button\" class=\"close\" data-dismiss=\"alert\" aria-label=\"Close\"><span aria-hidden=\"true\">×</span></button>
                      If you use Planner Arena or the OMPL benchmarking facilities, then we kindly ask you to include the following citation in your publications:
                     <blockquote>
                        Mark Moll, Ioan A. Șucan, Lydia E. Kavraki, <a href=\"https://moll.ai/publications/moll2015benchmarking-motion-planning-algorithms.pdf\">Benchmarking Motion Planning Algorithms: An Extensible Infrastructure for Analysis and Visualization</a>, <em>IEEE Robotics & Automation Magazine,</em> 22(3):96–102, September 2015. doi: <a href=\"https://dx.doi.org/10.1109/MRA.2015.2448276\">10.1109/MRA.2015.2448276</a>.
                      </blockquote>
                    </div>"
                ),
                downloadButton("perf_download_plot", "Download plot as PDF"),
                downloadButton("perf_download_rdata", "Download plot as RData"),
                plotOutput("perf_plot"),
                h4("Number of missing data points out of the total number of runs per planner"),
                tableOutput("perf_missing_data_table")
            )
        )
    })
    output$progress_page <- renderUI({
        validate(need(con(), no_database_text))
        validate(need(progress() %>% tally() > 0,
                      "There is no progress data in this database."))
        sidebarLayout(
            sidebarPanel(
                uiOutput("prog_problem_select"),
                uiOutput("prog_problem_param_select"),
                uiOutput("prog_attr_select"),
                uiOutput("prog_version_select"),
                uiOutput("prog_planner_select")
            ),
            mainPanel(
                downloadButton("prog_download_plot", "Download plot as PDF"),
                downloadButton("prog_download_rdata", "Download plot as RData"),
                plotOutput("prog_plot"),
                plotOutput("prog_num_measurements_plot")
            )
        )
    })

    output$regression_page <- renderUI({
        validate(need(con(), no_database_text))
        validate(need(experiments() %>% select(version) %>% distinct() %>%
                      tally() > 1,
            "Only one version of OMPL was used for the benchmarks."))
        sidebarLayout(
            sidebarPanel(
                uiOutput("regr_problem_select"),
                uiOutput("regr_problem_param_select"),
                uiOutput("regr_attr_select"),
                uiOutput("regr_version_select"),
                uiOutput("regr_planner_select")
            ),
            mainPanel(
                downloadButton("regr_download_plot", "Download plot as PDF"),
                downloadButton("regr_download_rdata", "Download plot as RData"),
                plotOutput("regr_plot")
            )
        )
    })

    output$dbinfo_page <- renderUI({
        validate(need(con(), no_database_text))
        tabsetPanel(
            tabPanel("Benchmark setup",  tableOutput("benchmark_info")),
            tabPanel("Planner Configurations", tableOutput("planner_configs"))
        )
    })
})
