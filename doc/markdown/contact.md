# Contact Us {#contact}

\htmlonly<div class="row"><div class="col-md-6 col-sm-6">\endhtmlonly
# Mailing Lists

 - Email ompl-users@lists.sourceforge.net to write to the OMPL discussion mailing list. You can subscribe to the list and see an archive of previous messages [here](https://lists.sourceforge.net/lists/listinfo/ompl-users).
 - Email ompl-devel@lists.sourceforge.net to contact the core developers. Please use this list only if you have questions that you feel are not appropriate for the ompl-users mailing list.

# Reporting Bugs

To report a bug, please click [here](https://bitbucket.org/ompl/ompl/issues?status=new&status=open).

\htmlonly</div><div class="col-md-6 col-sm-6">\endhtmlonly
# Contact form

\htmlonly
<form class="form" role="form" action="http://formspree.io/mmoll@rice.edu" method="POST">
  <div class="form-group">
    <input type="text" class="form-control" name="name" value='' placeholder="Your name">
  </div>
  <div class="form-group">
    <input type="email" class="form-control" name="_replyto" value='' placeholder="Your email (optional)">
  </div>
  <div class="form-group">
    <textarea id="message" name="message" rows=6 class="form-control" placeholder="Your message"></textarea>
  </div>
  <input type="hidden" name="_next" value="thank-you.html" />
  <input type="hidden" name="_subject" value="OMPL contact form submission" />
  <input type="hidden" name="IP" id="IP">
  <script type="application/javascript">
    window.onload = function () {
    $.getJSON("https://api.ipify.org?format=jsonp&callback=?",function(json){$("#IP").val(json.ip);});};
  </script>
  <input type="text" name="_gotcha" style="display:none" />
  <input type="submit" value="Send" name='submit' class="btn btn-primary" />
</form>
</div>
</div>
\endhtmlonly
