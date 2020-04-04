# Get Support {#support}

\htmlonly<div class="row"><div class="col-lg-6 col-md-12">\endhtmlonly

## Troubleshooting Information

- Check out the list of [Frequently Asked Questions](FAQ.html) to see if your question has been answered already.
- Check if your question has already been answered on the ompl-users mailing by searching the [mailing list archive](https://sourceforge.net/p/ompl/mailman/ompl-users/). You can also [subscribe to the user mailing list](https://lists.sourceforge.net/lists/listinfo/ompl-users).
- Check if the issue you are dealing with has already been reported on the [issue tracker on GitHub](https://github.com/ompl/ompl/issues?utf8=✓&q=is%3Aissue). If it is not, consider creating a new issue.
- Email ompl-devel@lists.sourceforge.net to contact the core developers. Please use this list only if you have questions that you feel are not appropriate for the ompl-users mailing list.


\htmlonly</div><div class="col-lg-6 col-md-12">\endhtmlonly

## Contact form

Use this form to contact the core OMPL developers directly.
\htmlonly
<form class="form" role="form" action="https://formspree.io/mmoll@rice.edu" method="POST">
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
