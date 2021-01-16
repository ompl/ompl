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

shinyUI(navbarPage("Planner Arena",
    useShinyjs(),
    extendShinyjs(script = "plannerarena.js", functions=c("shinyjs.refresh")),
    tabPanel("Overall performance",
        uiOutput("performance_page"),
        value = "performance",
        icon = icon("bar-chart")),
    tabPanel("Progress",
        uiOutput("progress_page"),
        value = "progress",
        icon = icon("area-chart")),
    tabPanel("Regression",
        uiOutput("regression_page"),
        value = "regression",
        icon = icon("bar-chart")),
    tabPanel("Database info",
        uiOutput("dbinfo_page"),
        value = "dbinfo",
        icon = icon("info-circle")),
    tabPanel("Change database",
        div(class = "row",
            div(class = "col-sm-10 col-sm-offset-1",
                fileInput("database",
                    label = h2("Upload benchmark database"),
                    accept = c("application/x-sqlite3", ".db")
                ),
                h2("Default benchmark database"),
                tags$ul(
                    tags$li(a(href = "javascript:history.go(0)",
                              "Reset to default database")),
                    tags$li(a(href = "benchmark.db",
                              "Download default database"))
                )
            )
        ),
        value = "database",
        icon = icon("database")),
    tabPanel("Help",
        div(class = "row",
            div(class = "col-sm-10 col-sm-offset-1",
                includeMarkdown("www/help.md")
            )
        ),
        value = "help",
        icon = icon("question-circle")),
    tabPanel("Settings",
        div(class = "row",
            div(class = "col-sm-10 col-sm-offset-1",
                h2("Plot settings"),
                h3("Font settings"),
                selectInput("fontFamily",
                        label = "Font family",
                        choices = c("Courier", "Helvetica", "Palatino",
                                    "Times"),
                        selected  = "Helvetica"),
                numericInput("fontSize", "Font size", 20, min = 1, max = 100),
                h3("PDF export paper size (in inches)"),
                numericInput("paperWidth", "Width", 12, min = 1, max = 50),
                numericInput("paperHeight", "Height", 8, min = 1, max = 50)
            )
        ),
        value = "settings",
        icon = icon("gear")),
    id = "navbar",
    header = tags$link(rel = "stylesheet", type = "text/css",
                       href = "plannerarena.css"),
    footer = div(class = "footer",
        div(class = "container",
                a(href = "http://kavrakilab.org",
                  "Kavraki Lab"),
                "•",
                a(href = "https://www.cs.rice.edu",
                  "Department of Computer Science"),
                "•",
                a(href = "https://www.rice.edu", "Rice University"),
                br(),
                "Funded in part by the",
                a(href = "https://www.nsf.gov",
                  "National Science Foundation")
        ),
        includeScript("www/ga.js")
    ),
    inverse = TRUE
))
