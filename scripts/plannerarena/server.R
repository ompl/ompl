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

defaultDatabase <- "www/benchmark.db"
noDatabaseText <- "No database loaded yet. Upload one by clicking on “Change database”."
notReadyText <- "The benchmarking results are not available yet, check back later."
sessionsFolder <- "/tmp/omplweb_sessions"
problemParamsAggregateText <- "all (aggregate)"
problemParamsSeparateText <- "all (separate)"

problemSelectWidget <- function(problems, widgetName) {
    widget <- selectInput(widgetName,
        label = h4("Motion planning problem"),
        choices = problems)
    conditionalDisable(widget, length(problems) < 2)
}

problemParamSelect <- function(param, val) {
    if (val == problemParamsAggregateText || val == problemParamsSeparateText)
        # select all
        1
    else
        # select specific parameter value.
        # Use fuzzy matching when comparing numbers because precision is lost
        # when real-valued parameter values are converted to strings for
        # parameter selection widget.
        if (regexpr('[-+]?\\d*\\.\\d+|\\d+', val)[1] == -1)
            sprintf('%s == %s', param, val)
        else
            sprintf('abs(%s - %s) < 0.0000001', param, val)
}

# return parameters of parametrized benchmarks if they exist, NULL otherwise
problemParams <- function(experiments) {
    params <- tbl_vars(experiments)
    numParams <- length(params)
    if (numParams > 12)
        paramNames <- params[13:numParams]
    else
        paramNames <- NULL
}
# return values of benchmark parameters
problemParamValues <- function(experiments, prefix, input) {
    params <- problemParams(experiments)
    values <- lapply(params,
        function(p) eval(parse(text=sprintf("input$%s", paste0(prefix, "problemParam", p)))))
    names(values) <- params
    return(values)
}
# determine whether a performance attribute should be grouped by a benchmark parameter value
problemParamGroupBy <- function(values) {
    grouping <- match(problemParamsSeparateText, values)
    if (is.na(grouping))
        NULL
    else
        names(values)[grouping]
}
# create a widget for a given benchmark parameter
problemParamSelectWidget <- function(name, experiments, prefix, problem, version) {
    values <- (experiments %>%
        filter(name == problem & version == version) %>%
        select_(name) %>% distinct() %>% collect())[[name]]
    dispName <- gsub("_", " ", name)
    internalName <- paste0(prefix, "problemParam", name)
    if (length(values)==1)
    {
        # don't show any widget for parameter if the only value is NA
        # (this means that the given benchmark parameter is not applicable to the
        # currently selected benchmark problem)
        if (!is.na(values[1]))
            # disable selection if there is only value for parameter
            disable(selectInput(internalName, label = h6(dispName), choices = values))
    }
    else
        selectInput(internalName, label = h6(dispName), choices = append(values,
            c(problemParamsAggregateText, problemParamsSeparateText), 0))
}
# create widgets for all benchmark parameters
problemParamSelectWidgets <- function(experiments, prefix, problem, version) {
    params <- problemParams(experiments)
    if (!is.null(params))
        # eparams <- paste0("experiment.", params)
        # names(eparams) <- gsub("_", " ", params)
        div(class="well well-light",
            h5("Problem parameters"),
            lapply(params, problemParamSelectWidget,
                experiments = experiments, prefix = prefix, problem = problem, version = version)
        )
}

versionSelectWidget <- function(experiments, widgetName, checkbox) {
    versions <- (experiments %>% select(version) %>% distinct() %>% collect())$version
    if (checkbox)
        widget <- checkboxGroupInput(widgetName, label = h4("Selected versions"),
            choices = versions,
            selected = versions)
    else
        widget <- selectInput(widgetName, label = h4("Version"),
            choices = versions,
            # select most recent version by default
            selected = tail(versions, n = 1))
    conditionalDisable(widget, length(versions) < 2)
}

plannerNameMapping <- function(fullname) {
    sub("control_", " ", sub("geometric_", "", fullname))
}
plannerSelectWidget <- function(performance, widgetName) {
    planners <- (performance %>% select(planner) %>% distinct() %>% collect())$planner
    names(planners) <- plannerNameMapping(planners)
    # select first 4 planners (or all if there are less than 4)
    if (length(planners) < 4)
        selection <- planners
    else
        selection <- planners[1:4]
    #conditionalDisable(
        checkboxGroupInput(widgetName, label = h4("Selected planners"),
        choices = planners, selected = selection)#, length(planners) < 2)
}

perfAttrSelectWidget <- function(runs, widgetName) {
    attrs <- tbl_vars(runs)
    names(attrs) <- gsub("_", " ", gsub("^run\\.", "", attrs))
    if ('time' %in% attrs)
        selection <- 'time'
    else
        selection <- NULL
    # strip off first 3 names, which correspond to internal id's
    selectInput(widgetName, label = h4("Benchmark attribute"),
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
                database <- defaultDatabase
            else
                # case 2
                database <- input$database$datapath
        } else
            # case 3
            database <- paste(sessionsFolder, query$user, query$job, sep="/")
        if (file.exists(database)) {
            con <- DBI::dbConnect(RSQLite::SQLite(), database)
            # benchmark job may not yet be finished so check that "experiments"
            # table exists.
            if ("experiments" %in% DBI::dbListTables(con))
            {
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
        con() %>%
        tbl("experiments") %>%
        rename(experiment.id = id) %>%
        rename(experiment = name)
    })
    plannerConfigs <- reactive({
        con() %>%
        tbl("plannerConfigs") %>%
        rename(planner.id = id) %>%
        rename(planner = name)
    })
    runs <- reactive({con() %>% tbl("runs") %>% rename(run.id = id)})
    attrs.names <- reactive({
        rnames <- tbl_vars(runs())
        names(rnames) <- gsub("_", " ", rnames)
        rnames
    })
    progress <- reactive({
        con() %>%
        tbl("progress") %>%
        rename(progress.time = time) %>%
        rename(progress.best_cost = best_cost)
    })
    progAttrs.names <- reactive({
        rnames <- tbl_vars(progress())
        names(rnames) <- gsub("_", " ", gsub("^progress\\.", "", rnames))
        rnames
    })
    enums <- reactive({con() %>% tbl("enums")})
    runs_ext <- reactive({plannerConfigs() %>%
        inner_join(runs(), c("planner.id" = "plannerid"), suffix = c(".planner", ".run")) %>%
        inner_join(experiments(), c("experimentid" = "experiment.id"), suffix = c(".plannerrun", ".experiment"))
    })
    performance <- reactive({runs_ext() %>%
        filter(experiment == input$perfProblem & version == input$perfVersion)
    })
    progPerf <- reactive({runs_ext() %>%
        inner_join(progress(), c("run.id" = "runid")) %>%
        filter(experiment == input$progProblem & version == input$progVersion)
    })
    regrPerf <- reactive({runs_ext() %>%
        filter(experiment == input$regrProblem & version %in% input$regrVersions)
    })

    # Go straight to the database upload page if there is no default database
    observe({
        if (is.null(con()))
            updateTabsetPanel(session, "navbar", selected = "database")
    })

    problemNames <- reactive({
        (experiments() %>% select(experiment) %>% distinct() %>% collect())$experiment })
    output$perfProblemSelect <- renderUI({ problemSelectWidget(problemNames(), "perfProblem") })
    output$progProblemSelect <- renderUI({ problemSelectWidget(problemNames(), "progProblem") })
    output$regrProblemSelect <- renderUI({ problemSelectWidget(problemNames(), "regrProblem") })

    output$perfProblemParamSelect <- renderUI({
        validate(
            need(input$perfProblem, 'Select a problem'),
            need(input$perfVersion, 'Select a version')
        )
        problemParamSelectWidgets(experiments(), "perf", input$perfProblem, input$perfVersion)
    })
    output$progProblemParamSelect <- renderUI({
        validate(
            need(input$progProblem, 'Select a problem'),
            need(input$progVersion, 'Select a version')
        )
        problemParamSelectWidgets(experiments(), "prog", input$progProblem, input$progVersion)
    })
    output$regrProblemParamSelect <- renderUI({
        validate(
            need(input$regrProblem, 'Select a problem'),
            need(input$regrVersions, 'Select a version')
        )
        problemParamSelectWidgets(experiments(), "regr", input$regrProblem, tail(input$regrVersions, n=1))
    })

    output$perfAttrSelect <- renderUI({
        tagList(
            perfAttrSelectWidget(runs(), "perfAttr"),
            checkboxInput('perfShowAdvOptions', 'Show advanced options', FALSE),
            conditionalPanel(condition = 'input.perfShowAdvOptions',
                div(class="well well-light",
                    checkboxInput("perfShowAsCDF", label = "Show as cumulative distribution function"),
                    checkboxInput("perfShowSimplified", label = "Include results after simplification"),
                    checkboxInput("perfShowParameterizedBoxPlots", label = "Show box plots for parametrized benchmarks"),
                    checkboxInput("perfHideOutliers", label = "Hide outliers in box plots"),
                    checkboxInput("perfYLogScale", label = "Use log scale for Y-axis")
                )
            )
        )
    })
    output$regrAttrSelect <- renderUI({
        perfAttrSelectWidget(runs(), "regrAttr")
    })
    output$progAttrSelect <- renderUI({
        progressAttrs <- tbl_vars(progress())
        names(progressAttrs) <- gsub("_", " ", progressAttrs)
        # strip off first 2 names, which correspond to an internal id and time
        attrs <- progressAttrs[3:length(progressAttrs)]
        tagList(
            conditionalDisable(selectInput("progress", label = h4("Progress attribute"),
                choices = attrs
            ), length(attrs) < 2),
            checkboxInput('progShowAdvOptions', 'Show advanced options', FALSE),
            conditionalPanel(condition = 'input.progShowAdvOptions',
                div(class="well well-light",
                    checkboxInput("progressShowMeasurements", label = "Show individual measurements"),
                    sliderInput("progressOpacity", label = "Measurement opacity", 0, 100, 50)
                )
            )
        )
    })

    output$perfVersionSelect <- renderUI({ versionSelectWidget(experiments(), "perfVersion", FALSE) })
    output$progVersionSelect <- renderUI({ versionSelectWidget(experiments(), "progVersion", FALSE) })
    output$regrVersionSelect <- renderUI({ versionSelectWidget(experiments(), "regrVersions", TRUE) })

    output$perfPlannerSelect <- renderUI({
        validate(
            need(input$perfProblem, 'Select a problem'),
            need(input$perfVersion, 'Select a version')
        )
        plannerSelectWidget(performance(), "perfPlanners")
    })
    output$progPlannerSelect <- renderUI({
        validate(
            need(input$progProblem, 'Select a problem'),
            need(input$progVersion, 'Select a version')
        )
        plannerSelectWidget(progPerf(), "progPlanners")
    })
    output$regrPlannerSelect <- renderUI({
        validate(
            need(input$regrProblem, 'Select a problem'),
            need(input$regrVersions, 'Select a version')
        )
        plannerSelectWidget(regrPerf(), "regrPlanners")
    })


    output$benchmarkInfo <- renderTable({
        validate(need(experiments(), noDatabaseText))
        validate(need(input$perfVersion, "Select a version on the “Overall performance” page"))
        experiments() %>%
            filter(experiment == input$perfProblem & version == input$perfVersion) %>%
            collect() %>% t()
    }, rownames=TRUE, colnames=FALSE)
    output$plannerConfigs <- renderTable({
        performance() %>%
            select(planner, settings) %>%
            distinct() %>%
            collect()
    }, rownames=FALSE, colnames=FALSE)

    # font selection
    fontSelection <- reactive({
        element_text(family = input$fontFamily, size = input$fontSize)
    })

    # plot of overall performance
    perfPlot <- reactive({
        # performance results for parametrized benchmarks can be grouped by parameter values
        paramValues <- problemParamValues(experiments(), "perf", input)
        grouping <- problemParamGroupBy(paramValues)
        # for certain performance metrics we can also include the results after path simplification
        attribs <- tbl_vars(runs())
        attr <- input$perfAttr
        dispAttr <- names(attrs.names()[attrs.names() == attr])
        simplifiedAttr <- gsub("^", "simplified_", attr)
        includeSimplifiedAttr <- input$perfShowSimplified && simplifiedAttr %in% attribs
        # compute the selection of columns and their new names
        selection <- c(planner = "planner", attr = attr)
        if (includeSimplifiedAttr)
            selection <- c(selection, "simplifiedAttr" = simplifiedAttr)
        if (!is.null(grouping))
            selection <- c(selection, "grouping" = grouping)
        # compute selection of rows (add empty string to work around bug if there is only one planner selected)
        filter_expr <- c(~ planner %in% c(input$perfPlanners,''))
        # for parametrized benchmarks we want only the data that matches all parameters exactly
        if (length(paramValues) > 0)
            filter_expr <- c(filter_expr, paste(mapply(
                problemParamSelect, names(paramValues), paramValues), collapse= " & "))
        # extract the data to be plotted
        data <- performance() %>%
            filter_(.dots = filter_expr) %>%
            select_(.dots = selection) %>%
            collect()
        # turn the planner and grouping columns into factors ordered in the same way that they occur in the database.
        uplanner <- unique(data$planner)
        data$planner <- factor(data$planner, uplanner,
            labels = sapply(uplanner, plannerNameMapping))
        if (!is.null(grouping))
            data$grouping <- factor(data$grouping)
        # use bar charts for enum types
        enum <- enums() %>% filter(name == attr) %>% collect()
        if (nrow(enum) > 0)
        {
            val <- enum$value
            names(val) <- enum$description
            attrAsFactor <- factor(data$attr, levels=val)
            levels(attrAsFactor) <- enum$description
            p <- qplot(planner, data = data, geom = "bar", fill = attrAsFactor) +
                # labels
                theme(legend.title = element_blank(), text = fontSelection())
            if (!is.null(grouping))
                p <- p + facet_grid(. ~ grouping)
        }
        else
        {
            if (input$perfHideOutliers)
                outlier.shape <- NA
            else
                outlier.shape <- 16
            if (includeSimplifiedAttr)
            {
                # the "all (separate)" case is not handled here
                data <- data %>%
                    collect() %>%
                    gather(key, value, c(attr, simplifiedAttr), factor_key=TRUE)
                if (input$perfShowAsCDF)
                    p <- ggplot(data, aes(x = value, color = planner,
                        group = interaction(planner, key), linetype=key)) +
                        # labels
                        xlab(input$perfAttr) +
                        ylab('cumulative probability') +
                        theme(text = fontSelection()) +
                        # empirical cumulative distribution function
                        stat_ecdf(size = 1) +
                        scale_linetype_discrete(name = "", labels = c("before simplification", "after simplification"))
                else
                {
                    p <- ggplot(data, aes(x=planner, y=value, color=key, fill=key)) +
                        # labels
                        ylab(dispAttr) +
                        theme(legend.title = element_blank(), text = fontSelection()) +
                        geom_boxplot(outlier.shape = outlier.shape) +
                        scale_fill_manual(values = c("#99c9eb", "#ebc999"),
                            labels = c("before simplification", "after simplification")) +
                        scale_color_manual(values =c("#3073ba", "#ba7330"),
                            labels = c("before simplification", "after simplification"))
                    if (input$perfHideOutliers)
                        lims <- quantile(data$value, c(0.2, 0.8))
                    else
                        lims <- c(NA, NA)
                    if (input$perfYLogScale)
                        p <- p + scale_y_log10(limits = lims)
                    else
                        p <- p + scale_y_continuous(limits = lims)
                }
            }
            else
            {
                if (input$perfShowAsCDF)
                {
                    if (is.null(grouping))
                        p <- ggplot(data, aes(x = attr, color = planner))
                    else
                        p <- ggplot(data, aes(x = attr, color = planner, linetype = grouping)) +
                            scale_linetype(name = gsub(" ", "_", grouping))
                    p <- p +
                        # labels
                        xlab(input$perfAttr) +
                        ylab('cumulative probability') +
                        theme(text = fontSelection()) +
                        # empirical cumulative distribution function
                        stat_ecdf(size = 1)
                } else {
                    if (input$perfShowParameterizedBoxPlots || is.null(grouping)) {
                        p <- ggplot(data, aes(x = planner, y = attr, group = planner)) +
                            # labels
                            ylab(dispAttr) +
                            theme(legend.position = "none", text = fontSelection()) +
                            # box plots for boolean, integer, and real-valued attributes
                            geom_boxplot(color = I("#3073ba"), fill = I("#99c9eb"), outlier.shape = outlier.shape)
                        if (!is.null(grouping))
                            p <- p + facet_grid(. ~ grouping)
                        if (input$perfHideOutliers)
                            lims <- quantile(data$value, c(0.2, 0.8))
                        else
                            lims <- c(NA, NA)
                    } else {
                        p <- ggplot(data, aes(x = planner, y = attr, color = grouping, fill = grouping)) +
                            # labels
                            ylab(dispAttr) +
                            theme(legend.title = element_blank(), text = fontSelection()) +
                            geom_boxplot(position = "dodge", outlier.shape = outlier.shape) +
                            scale_x_discrete(expand=c(0.05,0))
                            if (input$perfHideOutliers)
                                lims <- quantile(data$attr, c(0.2, 0.8))
                            else
                                lims <- c(NA, NA)
                    }
                    if (input$perfYLogScale)
                        p <- p + scale_y_log10(limits = lims)
                    else
                        p <- p + scale_y_continuous(limits = lims)
                }
            }
        }
        p
    })
    output$perfPlot <- renderPlot({
        validate(
            need(input$perfVersion, 'Select a version'),
            need(input$perfProblem, 'Select a problem'),
            need(input$perfAttr, 'Select a benchmark attribute'),
            need(input$perfPlanners, 'Select some planners')
        )
        print(perfPlot())
    })
    output$perfDownloadPlot <- downloadHandler(filename = 'perfplot.pdf',
        content = function(file) {
            pdf(file=file, width=input$paperWidth, height=input$paperHeight)
            print(perfPlot())
            dev.off()
        }
    )
    output$perfDownloadRdata <- downloadHandler(filename = 'perfplot.RData',
        content = function(file) {
            save(perfplot(), file = file)
        }
    )
    output$perfMissingDataTable <- renderTable({
        validate(
            need(input$perfVersion, 'Select a version'),
            need(input$perfProblem, 'Select a problem'),
            need(input$perfAttr, 'Select a benchmark attribute'),
            need(input$perfPlanners, 'Select some planners')
        )
        attr <- input$perfAttr
        # performance results for parametrized benchmarks can be grouped by parameter values
        paramValues <- problemParamValues(experiments(), "perf", input)
        grouping <- problemParamGroupBy(paramValues)
        if (!is.null(grouping))
            group_order <- c("planner", grouping)
        else
            group_order <- c("planner")
        data <- performance() %>%
            # add empty string to work around bug if there is only one planner selected
            filter_(.dots = ~ planner %in% c(input$perfPlanners,'')) %>%
            group_by_(.dots = group_order) %>%
            select_(.dots = c("attr" = attr)) %>%
            mutate_(missing = ~ is.na(attr)) %>%
            summarise(missing = sum(missing), total = n()) %>%
            collect()
        data$planner <- factor(data$planner, unique(data$planner), labels = sapply(unique(data$planner), plannerNameMapping))
        data
    }, rownames=FALSE)

    # progress plot
    progPlotData <- reactive({
        validate(
            need(input$progVersion, 'Select a version'),
            need(input$progProblem, 'Select a problem'),
            need(input$progress, 'Select a benchmark attribute'),
            need(input$progPlanners, 'Select some planners')
        )
        # performance results for parametrized benchmarks can be grouped by parameter values
        paramValues <- problemParamValues(experiments(), "prog", input)
        grouping <- problemParamGroupBy(paramValues)
        attr <- gsub(" ", "_", input$progress)
        selection <- c("planner", "time" = "progress.time", "attr" = attr)
        if (!is.null(grouping))
            selection <- c(selection, "grouping" = grouping)
        # compute selection of rows (add empty string to work around bug if there is only one planner selected)
        filter_expr <- c(~ planner %in% c(input$progPlanners,''), ~ !is.na(attr))
        if (length(paramValues) > 0)
            filter_expr <- c(filter_expr, paste(mapply(
                problemParamSelect, names(paramValues), paramValues), collapse= " & "))
        # extract the data to be plotted
        data <- progPerf() %>%
            filter_(.dots = filter_expr) %>%
            select_(.dots = selection) %>%
            collect()

        # turn the planner and grouping columns into factors ordered in the same way that they occur in the database.
        data$planner <- factor(data$planner, unique(data$planner),
            labels = sapply(unique(data$planner), plannerNameMapping))
        if (!is.null(grouping))
            data$grouping <- factor(data$grouping)
        list(data = data, grouping = grouping)
    })
    progPlot <- reactive({
        attr <- gsub(" ", "_", input$progress)
        dispAttr <- names(progAttrs.names()[progAttrs.names() == attr])
        progdata <- progPlotData()
        data <- progdata$data
        grouping <- progdata$grouping
        validate(need(nrow(data) > 0, 'No progress data available; select a different benchmark, progress attribute, or planners.'))
        p <- ggplot(data, aes(x = time, y = attr, group = planner, color = planner, fill = planner)) +
            # labels
            xlab('time (s)') +
            ylab(dispAttr) +
            theme(text = fontSelection()) +
            # smooth interpolating curve
            geom_smooth(method = "gam") +
            coord_cartesian(xlim = c(0, trunc(max(data$time))))
        # optionally, add individual measurements as semi-transparent points
        if (input$progressShowMeasurements)
            p <- p + geom_point(alpha=I(input$progressOpacity / 100))
        if (!is.null(grouping))
            p <- p + facet_grid(grouping ~ .)
        p
    })
    output$progPlot <- renderPlot({ progPlot() })
    progNumMeasurementsPlot <- reactive({
        attr <- gsub(" ", "_", input$progress)
        dispAttr <- names(progAttrs.names()[progAttrs.names() == attr])
        progdata <- progPlotData()
        data <- progdata$data
        grouping <- progdata$grouping
        if (nrow(data) > 0)
        {
            p <- ggplot(data, aes(x = time, group = planner, color = planner)) +
                # labels
                xlab('time (s)') +
                ylab(sprintf("# measurements for %s", dispAttr)) +
                theme(text = fontSelection()) +
                geom_freqpoly(binwidth=1) +
                coord_cartesian(xlim = c(0, trunc(max(data$time))))
            if (!is.null(grouping))
                p <- p + facet_grid(grouping ~ .)
            p
        }
    })
    output$progNumMeasurementsPlot <- renderPlot({ progNumMeasurementsPlot() })
    output$progDownloadPlot <- downloadHandler(filename = 'progplot.pdf',
        content = function(file) {
            pdf(file=file, width=input$paperWidth, height=input$paperHeight)
            print(progPlot())
            print(progNumMeasurementsPlot())
            dev.off()
        }
    )
    output$progDownloadRdata <- downloadHandler(filename = 'progplot.RData',
        content = function(file) {
            progplot <- progPlot()
            prognummeasurementsplot <- progNumMeasurementsPlot()
            save(progplot, prognummeasurementsplot, file = file)
        }
    )

    # regression plot
    regrPlot <- reactive({
        paramValues <- problemParamValues(experiments(), "regr", input)
        grouping <- problemParamGroupBy(paramValues)
        attr <- input$regrAttr
        dispAttr <- names(attrs.names()[attrs.names() == attr])
        # compute the selection of columns and their new names
        selection <- c("planner", "attr" = attr, "version")
        if (!is.null(grouping))
            selection <- c(selection, "grouping" = grouping)
        # compute selection of rows (add empty string to work around bug if there is only one planner selected)
        filter_expr <- c(~ planner %in% c(input$regrPlanners,''),
                         ~ version %in% c(input$regrVersions,''))
        # for parametrized benchmarks we want only the data that matches all parameters exactly
        if (length(paramValues) > 0)
            filter_expr <- c(filter_expr, paste(mapply(
                problemParamSelect, names(paramValues), paramValues), collapse= " & "))
        # extract the data to be plotted
        data <- regrPerf() %>%
            filter_(.dots = filter_expr) %>%
            select_(.dots = selection) %>%
            collect()
        # turn the planner and grouping columns into factors ordered in the same way that they occur in the database.
        data$planner <- factor(data$planner, unique(data$planner),
            labels = sapply(unique(data$planner), plannerNameMapping))
        if (!is.null(grouping))
            data$grouping <- factor(data$grouping)
        # strip "OMPL " prefix, so we can fit more labels on the X-axis
        data$version <- sapply(data$version,
            function(str) {
                # assume the version number is the last "word" in the string
                tail(strsplit(str, " ")[[1]], n=1)
            })
        # order by order listed in data frame (i.e., "0.9.*" before "0.10.*")
        data$version <- factor(data$version, unique(data$version))
        p <- ggplot(data, aes(x = version, y = attr, fill = planner, group = planner)) +
            # labels
            ylab(dispAttr) +
            theme(legend.title = element_blank(), text = fontSelection()) +
            # plot mean and error bars
            stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge()) +
            stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
        if (!is.null(grouping))
            p <- p + facet_grid(grouping ~ .)
        list(plot = p, query = query)
    })
    output$regrPlot <- renderPlot({
        validate(
            need(input$regrVersions, 'Select a version'),
            need(input$regrProblem, 'Select a problem'),
            need(input$regrAttr, 'Select a benchmark attribute'),
            need(input$regrPlanners, 'Select some planners')
        )
        print(regrPlot()$plot)
    })
    output$regrDownloadPlot <- downloadHandler(filename = 'regrplot.pdf',
        content = function(file) {
            pdf(file=file, width=input$paperWidth, height=input$paperHeight)
            print(regrPlot()$plot)
            dev.off()
        }
    )
    output$regrDownloadRdata <- downloadHandler(filename = 'regrplot.RData',
        content = function(file) {
            regrplot <- regrPlot()$plot
            save(regrplot, file = file)
        }
    )

    output$performancePage <- renderUI({
        validate(need(performance(), noDatabaseText))
        sidebarLayout(
            sidebarPanel(
                uiOutput("perfProblemSelect"),
                uiOutput("perfProblemParamSelect"),
                uiOutput("perfAttrSelect"),
                uiOutput("perfVersionSelect"),
                uiOutput("perfPlannerSelect")
            ),
            mainPanel(
                downloadButton('perfDownloadPlot', 'Download plot as PDF'),
                downloadButton('perfDownloadRdata', 'Download plot as RData'),
                plotOutput("perfPlot"),
                h4("Number of missing data points out of the total number of runs per planner"),
                tableOutput("perfMissingDataTable")
            )
        )
    })
    output$progressPage <- renderUI({
        validate(need(con(), noDatabaseText))
        validate(need(progress() %>% tally() %>% collect() > 0, "There is no progress data in this database."))
        sidebarLayout(
            sidebarPanel(
                uiOutput("progProblemSelect"),
                uiOutput("progProblemParamSelect"),
                uiOutput("progAttrSelect"),
                uiOutput("progVersionSelect"),
                uiOutput("progPlannerSelect")
            ),
            mainPanel(
                downloadButton('progDownloadPlot', 'Download plot as PDF'),
                downloadButton('progDownloadRdata', 'Download plot as RData'),
                plotOutput("progPlot"),
                plotOutput("progNumMeasurementsPlot")
            )
        )
    })

    output$regressionPage <- renderUI({
        validate(need(con(), noDatabaseText))
        validate(need(experiments() %>% select(version) %>% distinct() %>% tally() %>% collect() > 1,
            "Only one version of OMPL was used for the benchmarks."))
        sidebarLayout(
            sidebarPanel(
                uiOutput("regrProblemSelect"),
                uiOutput("regrProblemParamSelect"),
                uiOutput("regrAttrSelect"),
                uiOutput("regrVersionSelect"),
                uiOutput("regrPlannerSelect")
            ),
            mainPanel(
                downloadButton('regrDownloadPlot', 'Download plot as PDF'),
                downloadButton('regrDownloadRdata', 'Download plot as RData'),
                plotOutput("regrPlot")
            )
        )
    })

    output$dbinfoPage <- renderUI({
        validate(need(con(), noDatabaseText))
        tabsetPanel(
            tabPanel("Benchmark setup",  tableOutput("benchmarkInfo")),
            tabPanel("Planner Configurations", tableOutput("plannerConfigs"))
        )
    })
})


